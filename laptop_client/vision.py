"""
vision.py  –  NURC 2026 Vision + Control Dashboard
=====================================================
Self-contained pygame window that:
  • Shows live video from either the laptop camera OR the ESP32-CAM UDP stream
  • Runs YOLO sign detection + QR decoding on every frame
  • Sends UDP commands to the motor ESP32 and the cam ESP32
  • Provides a full keyboard control surface (see KEY MAP below)

Model format auto-detection (pass --model <path>):
  .pt          → PyTorch weights
  .onnx        → ONNX runtime
  <directory>  → OpenVINO  (directory must contain model.xml)

Sources  (pass --source <value>):
  0, 1, …   → local camera index  (fix for black-strip: uses cv2.VideoCapture)
  udp        → ESP32-CAM UDP JPEG stream on UDP_CAM_PORT (default)

KEY MAP
-------
  W / A / S / D   → motor: Forward / Left / Back / Right
  F                → toggle ESP-CAM flash
  E                → start fake search sequence (spin + scan)
  Q                → force-stop toggle (halt motors / resume)
  Arrow keys       → pan/tilt cam servos (UP/DOWN/LEFT/RIGHT)
  C                → reset cam servos to centre
  1-7              → show city info overlay (Tanger … Lagouira)
  ESC              → quit

Usage
-----
  python vision.py --source 0   --model ../model.onnx
  python vision.py --source udp --model ../openvino_model
  python vision.py              # uses defaults: udp + ../model.pt
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pygame
from pyzbar.pyzbar import decode
from ultralytics import YOLO

# ---------------------------------------------------------------------------
# Network configuration  (update IPs to match your ESP32 access points)
# ---------------------------------------------------------------------------
ESP32_MOTOR_IP     = "192.168.4.1"
ESP32_MOTOR_PORT   = 5001

ESP32_CAM_IP       = "192.168.4.2"
ESP32_CAM_CMD_PORT = 5002   # for control commands (flash, servo, ping)
ESP32_CAM_VID_PORT = 5000   # for incoming JPEG video frames

# ---------------------------------------------------------------------------
# Camera servo centre positions (microseconds or 0-180 angle, match firmware)
# ---------------------------------------------------------------------------
CAM_PAN_CENTER  = 90
CAM_TILT_CENTER = 90
CAM_STEP        = 10   # degrees per arrow-key press

# ---------------------------------------------------------------------------
# City info (keys 1-7)
# ---------------------------------------------------------------------------
CITIES = [
    ("Tanger",    "Porte de l'Afrique, carrefour des civilisations"),
    ("Fès",       "Première université au monde, berceau du savoir"),
    ("Rabat",     "Capitale royale et cité des remparts millénaires"),
    ("Marrakech", "La ville ocre, joyau impérial du Sud"),
    ("Tan-Tan",   "Porte du Sahara et ville de la transhumance"),
    ("Laâyoune",  "Capitale du Sahara marocain, cité du désert"),
    ("Lagouira",  "Pointe la plus méridionale du royaume chérifien"),
]

# ---------------------------------------------------------------------------
# VisionSystem – YOLO + QR wrapper
# ---------------------------------------------------------------------------
CONF_THRESHOLD = 0.5
IMG_SIZE       = 320

class VisionSystem:
    """Loads a YOLO model (auto-detecting format) and runs inference + QR scan."""

    def __init__(self, model_path: str | Path):
        model_path = Path(model_path)

        if model_path.is_dir():
            if not list(model_path.glob("*.xml")):
                raise FileNotFoundError(f"No .xml found in OpenVINO dir: {model_path}")
            load_path = str(model_path)
            fmt = "OpenVINO"
        elif model_path.suffix == ".onnx":
            load_path = str(model_path)
            fmt = "ONNX"
        elif model_path.suffix in (".pt", ".pth"):
            load_path = str(model_path)
            fmt = "PyTorch"
        else:
            raise ValueError(f"Unsupported model: {model_path}")

        print(f"[Vision] Loading {fmt} model: {load_path}")
        self.model = YOLO(load_path)
        print(f"[Vision] Classes: {list(self.model.names.values())}")

    def analyze_frame(
        self, frame: np.ndarray
    ) -> tuple[np.ndarray, list[str], list[dict]]:
        """
        Returns (annotated_frame, qr_texts, yolo_detections).
        yolo_detections: list of {name, confidence, bbox}
        """
        # QR detection
        qr_data: list[str] = []
        for obj in decode(frame):
            pts = obj.polygon
            if len(pts) == 4:
                arr = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [arr], True, (255, 0, 0), 2)
            text = obj.data.decode("utf-8")
            qr_data.append(text)
            r = obj.rect
            cv2.putText(frame, text, (r.left, r.top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # YOLO detection
        yolo_results: list[dict] = []
        for res in self.model(frame, imgsz=IMG_SIZE, conf=CONF_THRESHOLD, verbose=False):
            for box in res.boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                conf   = float(box.conf[0])
                name   = self.model.names[int(box.cls[0])]
                yolo_results.append({"name": name, "confidence": conf,
                                     "bbox": (x1, y1, x2, y2)})
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{name} {conf:.2f}", (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        return frame, qr_data, yolo_results


# ---------------------------------------------------------------------------
# Network helpers
# ---------------------------------------------------------------------------

class Network:
    """Manages sockets + heartbeat for motor ESP and cam ESP."""

    def __init__(self):
        self.motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cam_sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.motor_sock.settimeout(0.8)
        self.cam_sock.settimeout(0.8)

        # Incoming UDP video socket
        self.vid_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vid_sock.bind(("0.0.0.0", ESP32_CAM_VID_PORT))
        self.vid_sock.settimeout(0.05)   # non-blocking-ish

        self.motor_ok = False
        self.cam_ok   = False

        # Latest decoded frame from UDP stream (thread-safe via lock)
        self._frame: np.ndarray | None = None
        self._frame_lock = threading.Lock()

        # Cam servo state
        self.pan  = CAM_PAN_CENTER
        self.tilt = CAM_TILT_CENTER

        # Force-stop flag
        self.force_stopped = False

        # Start heartbeat + video receiver threads
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        threading.Thread(target=self._video_recv_loop, daemon=True).start()

    # -- Internal threads --------------------------------------------------

    def _heartbeat_loop(self):
        while True:
            self.motor_ok = self._ping(self.motor_sock, ESP32_MOTOR_IP, ESP32_MOTOR_PORT)
            self.cam_ok   = self._ping(self.cam_sock,   ESP32_CAM_IP,   ESP32_CAM_CMD_PORT)
            time.sleep(0.5)

    @staticmethod
    def _ping(sock: socket.socket, ip: str, port: int) -> bool:
        try:
            sock.sendto(b"PING", (ip, port))
            data, _ = sock.recvfrom(1024)
            return b"PONG" in data
        except (socket.timeout, OSError):
            return False

    def _video_recv_loop(self):
        """Continuously receive UDP JPEG frames from ESP32-CAM."""
        while True:
            try:
                data, _ = self.vid_sock.recvfrom(65507)
                arr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    with self._frame_lock:
                        self._frame = frame
            except socket.timeout:
                pass
            except OSError:
                break

    def get_udp_frame(self) -> np.ndarray | None:
        with self._frame_lock:
            return self._frame.copy() if self._frame is not None else None

    # -- Command senders ---------------------------------------------------

    def send_motor(self, cmd: str):
        try:
            self.motor_sock.sendto(cmd.encode(), (ESP32_MOTOR_IP, ESP32_MOTOR_PORT))
        except OSError:
            pass

    def send_cam_cmd(self, cmd: str):
        try:
            self.cam_sock.sendto(cmd.encode(), (ESP32_CAM_IP, ESP32_CAM_CMD_PORT))
        except OSError:
            pass

    def toggle_flash(self):
        self.send_cam_cmd("FLASH_TOGGLE")

    def reset_servo(self):
        self.pan  = CAM_PAN_CENTER
        self.tilt = CAM_TILT_CENTER
        self._push_servo()

    def move_servo(self, dpan: int, dtilt: int):
        self.pan  = max(0, min(180, self.pan  + dpan))
        self.tilt = max(0, min(180, self.tilt + dtilt))
        self._push_servo()

    def _push_servo(self):
        self.send_cam_cmd(f"SERVO:{self.pan}:{self.tilt}")

    def toggle_force_stop(self) -> bool:
        self.force_stopped = not self.force_stopped
        if self.force_stopped:
            self.send_motor("S")
        return self.force_stopped

    def run_search_sequence(self):
        """Non-blocking fake autonomous search (runs in a thread)."""
        threading.Thread(target=self._search_seq, daemon=True).start()

    def _search_seq(self):
        """Spin slowly while scanning then stop."""
        steps = [("L", 0.4), ("S", 0.2), ("R", 0.4), ("S", 0.2),
                 ("L", 0.4), ("S", 0.2), ("R", 0.4), ("S", 0.5)]
        for cmd, duration in steps:
            if self.force_stopped:
                break
            self.send_motor(cmd)
            time.sleep(duration)
        self.send_motor("S")


# ---------------------------------------------------------------------------
# HUD renderer
# ---------------------------------------------------------------------------

COLORS = {
    "green":  (80,  220, 100),
    "red":    (220, 60,  60),
    "yellow": (255, 220, 0),
    "cyan":   (0,   220, 220),
    "orange": (255, 160, 30),
    "white":  (230, 230, 230),
    "bg":     (10,  10,  25),
    "panel":  (20,  20,  45, 180),   # RGBA for semi-transparent overlay
}


def _render_text(surf: pygame.Surface, font: pygame.font.Font,
                 text: str, pos: tuple[int, int],
                 color: tuple = COLORS["white"]) -> int:
    """Blit text and return the Y position after this line."""
    s = font.render(text, True, color)
    surf.blit(s, pos)
    return pos[1] + s.get_height() + 2


def render_hud(
    surf: pygame.Surface,
    font: pygame.font.Font,
    small_font: pygame.font.Font,
    net: Network,
    current_cmd: str,
    qr_data: list[str],
    signs: list[dict],
    city_info: str | None,
    force_stopped: bool,
):
    sw, sh = surf.get_size()

    # ---- Left panel (status) ----
    panel_w = 260
    panel_surf = pygame.Surface((panel_w, sh), pygame.SRCALPHA)
    panel_surf.fill((10, 10, 30, 170))
    surf.blit(panel_surf, (0, 0))

    x, y = 10, 10
    y = _render_text(surf, font,
                     f"MOTORS : {'✓ OK' if net.motor_ok else '✗ OFFLINE'}",
                     (x, y),
                     COLORS["green"] if net.motor_ok else COLORS["red"])
    y = _render_text(surf, font,
                     f"CAM    : {'✓ OK' if net.cam_ok else '✗ OFFLINE'}",
                     (x, y),
                     COLORS["green"] if net.cam_ok else COLORS["red"])
    y += 6
    cmd_color = COLORS["red"] if force_stopped else COLORS["yellow"]
    stop_txt  = "  [FORCE STOP]" if force_stopped else ""
    y = _render_text(surf, font, f"CMD: {current_cmd}{stop_txt}", (x, y), cmd_color)

    y += 4
    y = _render_text(surf, small_font,
                     f"PAN:{net.pan}°  TILT:{net.tilt}°", (x, y), COLORS["cyan"])

    # Detected signs
    y += 8
    if signs:
        y = _render_text(surf, font, "PANNEAUX DÉTECTÉS:", (x, y), COLORS["cyan"])
        for s in signs[:5]:
            y = _render_text(surf, small_font,
                             f"  {s['name']}  {s['confidence']*100:.0f}%",
                             (x, y), COLORS["white"])

    # QR codes
    if qr_data:
        y += 4
        y = _render_text(surf, font, "QR CODES:", (x, y), COLORS["orange"])
        for q in qr_data[:3]:
            y = _render_text(surf, small_font, f"  {q[:28]}", (x, y), COLORS["white"])

    # ---- City overlay (bottom centre) ----
    if city_info:
        city_surf = pygame.Surface((sw - panel_w - 20, 44), pygame.SRCALPHA)
        city_surf.fill((10, 10, 30, 200))
        surf.blit(city_surf, (panel_w + 10, sh - 54))
        _render_text(surf, font, city_info, (panel_w + 20, sh - 50), COLORS["yellow"])

    # ---- Key reference (bottom right) ----
    hint_lines = [
        "WASD: motors   Arrows: cam servo",
        "F: flash  E: search  Q: stop toggle",
        "C: reset cam  1-7: city  ESC: quit",
    ]
    hy = sh - len(hint_lines) * 18 - 6
    for line in hint_lines:
        _render_text(surf, small_font, line, (panel_w + 10, hy), (120, 120, 145))
        hy += 18


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run_dashboard(source: str, model_path: str):
    print("[Vision] Initialising pygame …")
    pygame.init()
    SW, SH = 900, 640
    screen = pygame.display.set_mode((SW, SH))
    pygame.display.set_caption("NURC 2026 – Vision & Control")
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont("verdana", 16, bold=True)
    sfont  = pygame.font.SysFont("verdana", 13)

    print("[Vision] Loading YOLO model …")
    vision = VisionSystem(model_path)

    print("[Vision] Starting network …")
    net = Network()

    # ---- Camera initialisation (local camera source) ----
    use_local_cam = source.lstrip("-").isdigit()
    cap: cv2.VideoCapture | None = None
    if use_local_cam:
        cam_idx = int(source)
        cap = cv2.VideoCapture(cam_idx)
        # Force a sane resolution so we don't get a black strip
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # minimise buffer lag
        if not cap.isOpened():
            print(f"[Vision] Cannot open camera {cam_idx}")
            sys.exit(1)
        # Warm up – discard first few frames (causes black strip on some drivers)
        for _ in range(5):
            cap.grab()
        print(f"[Vision] Camera {cam_idx} ready.")
    else:
        print(f"[Vision] UDP video auto-connect on port {ESP32_CAM_VID_PORT} …")

    # Dashboard state
    current_cmd  = "S"
    city_info: str | None = None
    city_timer   = 0.0

    running = True
    while running:
        now = time.time()

        # ---- Event handling ----
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                k = event.key

                # Quit
                if k == pygame.K_ESCAPE:
                    running = False

                # Flash toggle
                elif k == pygame.K_f:
                    net.toggle_flash()

                # Fake search sequence
                elif k == pygame.K_e:
                    if not net.force_stopped:
                        net.run_search_sequence()

                # Force stop toggle
                elif k == pygame.K_q:
                    net.toggle_force_stop()

                # Camera servo – arrow keys
                elif k == pygame.K_LEFT:
                    net.move_servo(-CAM_STEP, 0)
                elif k == pygame.K_RIGHT:
                    net.move_servo(+CAM_STEP, 0)
                elif k == pygame.K_UP:
                    net.move_servo(0, +CAM_STEP)
                elif k == pygame.K_DOWN:
                    net.move_servo(0, -CAM_STEP)

                # Camera reset
                elif k == pygame.K_c:
                    net.reset_servo()

                # City info  (1-7)
                elif pygame.K_1 <= k <= pygame.K_7:
                    idx = k - pygame.K_1          # 0-6
                    name, phrase = CITIES[idx]
                    city_info  = f"{name} — {phrase}"
                    city_timer = now + 5.0        # show for 5 s

        # Expire city overlay
        if city_info and now > city_timer:
            city_info = None

        # ---- Motor commands (key polling, so held keys work) ----
        if not net.force_stopped:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]:        cmd = "F"
            elif keys[pygame.K_s]:      cmd = "B"
            elif keys[pygame.K_a]:      cmd = "L"
            elif keys[pygame.K_d]:      cmd = "R"
            else:                       cmd = "S"

            if cmd != current_cmd:
                current_cmd = cmd
                net.send_motor(cmd)
        else:
            current_cmd = "STOP"

        # ---- Grab frame ----
        frame: np.ndarray | None = None
        if use_local_cam and cap is not None:
            # Use grab()+retrieve() to always get the freshest frame
            cap.grab()
            ret, raw = cap.retrieve()
            if ret and raw is not None and raw.size > 0:
                frame = raw
        else:
            frame = net.get_udp_frame()

        # ---- Background ----
        screen.fill(COLORS["bg"])

        # ---- Video + inference ----
        qr_data: list[str]  = []
        signs:   list[dict] = []

        if frame is not None:
            annotated, qr_data, signs = vision.analyze_frame(frame)

            # Convert BGR → RGB, resize to fill the right portion
            rgb = cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB)
            rgb = cv2.resize(rgb, (SW, SH))
            # OpenCV [H,W,C] → pygame [W,H,C]
            rgb = np.swapaxes(rgb, 0, 1)
            pg_surf = pygame.surfarray.make_surface(rgb)
            screen.blit(pg_surf, (0, 0))
        else:
            # No video – show a centred message
            msg = sfont.render(
                "En attente du flux vidéo …" if not use_local_cam
                else "Caméra non disponible",
                True, (160, 160, 180))
            screen.blit(msg, (SW // 2 - msg.get_width() // 2,
                              SH // 2 - msg.get_height() // 2))

        # ---- HUD overlay ----
        render_hud(screen, font, sfont, net, current_cmd,
                   qr_data, signs, city_info, net.force_stopped)

        pygame.display.flip()
        clock.tick(30)

    # ---- Cleanup ----
    print("[Vision] Shutting down …")
    net.send_motor("S")
    if cap:
        cap.release()
    pygame.quit()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="NURC 2026 Vision & Control Dashboard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--source", default="udp",
        help="Video source: integer camera index (e.g. 0) or 'udp'. Default: udp",
    )
    p.add_argument(
        "--model", default="../model.pt",
        help="Model path: .pt, .onnx, or OpenVINO directory. Default: ../model.pt",
    )
    return p


if __name__ == "__main__":
    args = _build_parser().parse_args()
    run_dashboard(source=args.source, model_path=args.model)
