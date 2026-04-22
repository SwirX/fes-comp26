import pygame
import socket
import cv2
import threading
import time
import numpy as np
from vision import VisionSystem

# Network Configurations
# WARNING: Update these IPs directly to what your ESP32 hotspots assign to them!
ESP32_MOTOR_IP = "192.168.4.1" 
ESP32_MOTOR_PORT = 5001

ESP32_CAM_IP = "192.168.4.2" 
ESP32_CAM_PORT = 5002

# Sockets for bidirectional UDP
motor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
motor_sock.settimeout(1.0)
cam_sock.settimeout(1.0)

# The video feed address (HTTP MJPEG chunk streamer)
video_url = f"http://{ESP32_CAM_IP}/"

# Dashboard global states
status = {
    "robot_connected": False,
    "cam_connected": False,
    "current_command": "S",
    "detected_qrs": [],
    "detected_signs": []
}

def network_heartbeat_thread():
    """
    Background Thread managing constant PING/PONG sequences assuring UDP 
    alive-capabilities and logging fail-safes.
    """
    while True:
        # Check Motors
        try:
            motor_sock.sendto(b"PING", (ESP32_MOTOR_IP, ESP32_MOTOR_PORT))
            data, _ = motor_sock.recvfrom(1024)
            if b"PONG" in data:
                status["robot_connected"] = True
        except socket.timeout:
            status["robot_connected"] = False
            
        # Check Camera Node Presence Protocol (Not the actual video stream, but WiFi check)
        try:
            cam_sock.sendto(b"PING", (ESP32_CAM_IP, ESP32_CAM_PORT))
            data, _ = cam_sock.recvfrom(1024)
            if b"PONG" in data:
                status["cam_connected"] = True
        except socket.timeout:
            status["cam_connected"] = False
            
        time.sleep(0.5)

def send_motor_command(cmd_char):
    if cmd_char != status["current_command"]:
        status["current_command"] = cmd_char
        try:
            # We enforce fire-and-forget UDP logic on motors
            motor_sock.sendto(cmd_char.encode('utf-8'), (ESP32_MOTOR_IP, ESP32_MOTOR_PORT))
        except Exception:
            pass

def render_hud(surface, font):
    """
    Builds the on-screen display overlay.
    """
    c_rob = (0, 255, 0) if status["robot_connected"] else (255, 0, 0)
    c_cam = (0, 255, 0) if status["cam_connected"] else (255, 0, 0)
    
    txt_rob = font.render(f"MOTORS: {'OK' if status['robot_connected'] else 'DISCONNECTED'}", True, c_rob)
    txt_cam = font.render(f"VIDEO LINK: {'OK' if status['cam_connected'] else 'DISCONNECTED'}", True, c_cam)
    txt_cmd = font.render(f"ACTION: {status['current_command']}", True, (255, 255, 0))
    
    surface.blit(txt_rob, (10, 10))
    surface.blit(txt_cam, (10, 40))
    surface.blit(txt_cmd, (10, 70))
    
    # Render Signs
    y_offset = 120
    if status["detected_signs"]:
        title = font.render("SIGNS DETECTED:", True, (0, 255, 255))
        surface.blit(title, (10, y_offset))
        y_offset += 30
        for sign in status["detected_signs"]:
            s_text = font.render(f"- {sign['name']} ({sign['confidence']*100:.0f}%)", True, (255, 255, 255))
            surface.blit(s_text, (20, y_offset))
            y_offset += 25

    # Render QRs
    y_offset += 10
    if status["detected_qrs"]:
        title = font.render("QR CODES DETECTED:", True, (255, 165, 0))
        surface.blit(title, (10, y_offset))
        y_offset += 30
        for qr in status["detected_qrs"][:3]: # limit amount shown
            q_text = font.render(f"- {qr}", True, (255, 255, 255))
            surface.blit(q_text, (20, y_offset))
            y_offset += 25

def main():
    print("Loading Pygame Engine...")
    pygame.init()
    screen_width, screen_height = 800, 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Fes-Comp2026 Control Center")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("verdana", 18, bold=True)
    
    # Launch UDP checking node background threading
    threading.Thread(target=network_heartbeat_thread, daemon=True).start()
    
    # Initialize UI / Compute logic
    print("Loading AI YOLO weights and PyZbar modules...")
    # Load specific trained weights if they exist (YOLOv8 automatically gracefully tracks paths)
    vision = VisionSystem(yolo_model_path="../yolo_training/runs/detect/moroccan_signs_model/weights/best.pt")
    
    print("Connecting to ESP32-CAM MJPEG Stream...")
    # Add small delay to let heartbeat resolve before blasting video stream grab
    time.sleep(1)
    cap = cv2.VideoCapture(video_url)
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                
        # --- KEYBOARD POLLING ---
        keys = pygame.key.get_pressed()
        if keys[pygame.K_w] or keys[pygame.K_UP]: send_motor_command("F")
        elif keys[pygame.K_s] or keys[pygame.K_DOWN]: send_motor_command("B")
        elif keys[pygame.K_a] or keys[pygame.K_LEFT]: send_motor_command("L")
        elif keys[pygame.K_d] or keys[pygame.K_RIGHT]: send_motor_command("R")
        else: send_motor_command("S")
        
        # --- VIDEO PIPELINE ---
        screen.fill((20, 20, 40)) # Dark blue background if video disconnected
        
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                # 1. Ask vision module to render annotations on top of the generic cv2 matrix
                frame, current_qrs, current_signs = vision.analyze_frame(frame)
                
                # 2. Update global statuses for HUD module
                status["detected_qrs"] = current_qrs
                status["detected_signs"] = current_signs
                
                # 3. Transform visual data for Pygame
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = cv2.resize(frame, (screen_width, screen_height))
                
                # Transform OpenCV arrays [H, W, C] to Pygame coordinate space [W, H, C]
                frame = np.swapaxes(frame, 0, 1)
                
                # 4. Generate the Pygame graphics surf
                pg_surface = pygame.surfarray.make_surface(frame)
                screen.blit(pg_surface, (0, 0))
        elif status["cam_connected"]:
            # If camera UDP responded but stream broke, attempt reload
            cap.open(video_url)
            
        # Draw all status widgets on top
        render_hud(screen, font)
        
        pygame.display.flip()
        
        # Limit the refresh rate computationally (fps=30)
        clock.tick(30)
        
    print("Shutting down... Killing Camera.")
    cap.release()
    pygame.quit()

if __name__ == "__main__":
    main()
