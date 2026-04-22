import cv2
import socket
import numpy as np
from ultralytics import YOLO

# 1. Configuration
UDP_IP = "0.0.0.0" # Écoute sur toutes les interfaces
UDP_PORT = 5000    # Port configuré sur l'ESP32-CAM
MODEL_PATH = "best.pt" # Ou best.onnx pour plus de vitesse

# 2. Initialisation
model = YOLO(MODEL_PATH)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"En attente du flux UDP sur le port {UDP_PORT}...")

buffer = b""

while True:
    # Recevoir les paquets UDP
    data, addr = sock.recvfrom(65507)
    
    # Reconstitution de l'image JPEG (si envoyée en un bloc)
    nparr = np.frombuffer(data, np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    if frame is not None:
        # Lancer la détection YOLO
        # imgsz=320 pour matcher l'entraînement, conf=0.5 pour éviter les faux positifs
        results = model.predict(frame, imgsz=320, conf=0.5, verbose=False)

        # Analyser les résultats
        for r in results:
            for box in r.boxes:
                class_id = int(box.cls[0])
                label = model.names[class_id]
                conf = box.conf[0]

                # Logique NURC spécifique :
                if label == "Stop":
                    print("🛑 STOP détecté ! Envoi commande arrêt...")
                    # Envoyer commande UDP vers l'ESP32 des moteurs ici
                
                if label == "Sens interdit":
                    print("⛔ Sens Interdit ! Correction trajectoire...")

        # Affichage (optionnel, consomme des ressources)
        annotated_frame = results[0].plot()
        cv2.imshow("NURC 2026 - Flux ESP32-CAM", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()