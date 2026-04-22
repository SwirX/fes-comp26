import os
from ultralytics import YOLO

def main():
    # Load a pretrained model (YOLOv8 nano is fastest for real-time edge performance)
    # Using 'yolo11n.pt' or 'yolov8n.pt' depending on what is downloaded
    model = YOLO("yolo11n.pt")  

    # Ensure dataset configuration exists
    config_path = "moroccan_signs.yaml"
    if not os.path.exists(config_path):
        print(f"Error: Could not find {config_path}")
        return

    print("Starting YOLO training for Moroccan road signs...")
    
    # Train the model
    # adjust 'epochs' and 'batch' size depending on your GPU resources
    results = model.train(
        data=config_path,
        epochs=100,
        imgsz=640,
        batch=16,
        name="moroccan_signs_model",
        device="auto" # Will automatically pick cuda if nvidia GPU is available
    )

    print("Training complete! Best model saved in runs/detect/moroccan_signs_model/weights/best.pt")

if __name__ == "__main__":
    main()
