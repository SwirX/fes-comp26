import cv2
import numpy as np
from pyzbar.pyzbar import decode
from ultralytics import YOLO

class VisionSystem:
    def __init__(self, yolo_model_path="yolo11n.pt"):
        """
        Initializes the vision system.
        If custom fine-tuned weights (e.g. from runs/detect/moroccan_signs_model/weights/best.pt)
        are provided and exist, they will be loaded. Otherwise defaults to standard YOLO nano.
        """
        try:
            self.model = YOLO(yolo_model_path)
        except Exception as e:
            print(f"Failed to load specific YOLO model ({e}). Falling back to 'yolo11n.pt'")
            self.model = YOLO("yolo11n.pt")
            
    def analyze_frame(self, frame):
        """
        Takes a BGR frame from OpenCV, runs QR decoding and YOLO inference.
        Returns the annotated BGR frame, a list of decoded QR texts, and a list of structured YOLO dicts.
        """
        # Optional: You might want to scale down frame to improve FPS on low-end laptops
        # frame = cv2.resize(frame, (640, 480))
        
        # --- 1. QR Code Detection ---
        qr_data = []
        decoded_objects = decode(frame)
        for obj in decoded_objects:
            # Draw polygon around the QR Outline
            pts = obj.polygon
            if len(pts) == 4:
                pts = np.array(pts, dtype=np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], isClosed=True, color=(255, 0, 0), thickness=2)
            
            # Decode the utf-8 text from QR
            text = obj.data.decode("utf-8")
            qr_data.append(text)
            
            # Position text above the QR
            rect = obj.rect
            cv2.putText(frame, text, (rect.left, rect.top - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        # --- 2. YOLO Road Sign Detection ---
        yolo_results = []
        # Inference (verbose=False avoids console spam per frame)
        results = self.model(frame, verbose=False)
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Box coordinates
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                
                # Confidence score tracking
                conf = box.conf[0].item()
                
                # Associated Class label
                cls_id = int(box.cls[0].item())
                name = self.model.names[cls_id]
                
                yolo_results.append({
                    "name": name,
                    "confidence": conf,
                    "bbox": (x1, y1, x2, y2)
                })
                
                # Draw Bounding Box and label payload onto the frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                label = f"{name} ({conf:.2f})"
                cv2.putText(frame, label, (x1, y1 - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            
        return frame, qr_data, yolo_results
