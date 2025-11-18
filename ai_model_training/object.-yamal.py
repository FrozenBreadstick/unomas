from ultralytics import YOLO

# load model
model = YOLO(r"C:\Users\AyberkYetkin\OneDrive - DroneShield\Documents\AI Drone\Code\segmentation\best.pt")

# Print number of classes
print(f"Number of classes: {len(model.names)}")

# Print class names
print("Class names:")
for i, name in model.names.items():
    print(f"{i}: {name}")

