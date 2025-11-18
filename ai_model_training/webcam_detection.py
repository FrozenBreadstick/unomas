import cv2
import torch
from ultralytics import YOLO
import threading
import time

CAM_INDEX = 0
CAM_CAPTURE_WIDTH = 1920
CAM_CAPTURE_HEIGHT = 1080
YOLO_WIDTH = 640
YOLO_HEIGHT = 480
DEVICE = "cuda:0"
BATCH_SIZE = 2
DRAW_EVERY_N_FRAMES = 0.0625
CONF_THRESHOLD = 0.15
IOU_THRESHOLD = 0.3

#Fframe Buffer 
class FrameBuffer:
    def __init__(self):
        self.frames = []
        self.lock = threading.Lock()

    def update(self, frame):
        with self.lock:
            if len(self.frames) >= BATCH_SIZE:
                self.frames = self.frames[-(BATCH_SIZE-1):]
            self.frames.append(frame)

    def get_batch(self):
        with self.lock:
            batch = self.frames
            self.frames = []
        return batch

# camera tred 
def camera_thread(buffer: FrameBuffer):
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_MSMF)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_CAPTURE_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 120)

    print("Camera actueal FPS:", cap.get(cv2.CAP_PROP_FPS))

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        frame_resized = cv2.resize(frame, (YOLO_WIDTH, YOLO_HEIGHT))
        buffer.update(frame_resized)

# detection loop
def main():
    model = YOLO(r"C:\Users\AyberkYetkin\OneDrive - DroneShield\Documents\AI Drone\Code\runs\detect\train36\weights\best.pt")
    model.to(DEVICE)

    buffer = FrameBuffer()
    t = threading.Thread(target=camera_thread, args=(buffer,), daemon=True)
    t.start()

    frame_count = 0
    start_time = time.time()
    fps_display = 0
    last_boxes = []

    while True:
        batch_frames = buffer.get_batch()
        if not batch_frames:
            continue

        batch_tensors = [torch.from_numpy(f).permute(2, 0, 1).float().cuda() / 255.0 for f in batch_frames]
        batch_tensors = torch.stack(batch_tensors)

        
        infer_start = time.time()
        results = model(batch_tensors, conf=CONF_THRESHOLD, iou=IOU_THRESHOLD)
        infer_time = time.time() - infer_start

        frame_count += len(batch_frames)

        for i, frame in enumerate(batch_frames):
            if i >= len(results):
                continue
            boxes = results[i].boxes
            if len(boxes) == 0 and last_boxes:
                for box in last_boxes:
                    x1, y1, x2, y2 = box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            else:
                last_boxes = [list(map(int, box.xyxy[0])) for box in boxes]
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = model.names[cls]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.putText(frame, f"FPS: {fps_display:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            if frame_count % DRAW_EVERY_N_FRAMES == 0:
                cv2.imshow("YOLOv8 Detetion RTX 4060", frame)

        # update FPS every seconds
        
        elapsed = time.time() - start_time
        if elapsed >= 1.0:
            fps_display = frame_count / elapsed
            print(f"Total FPS: {fps_display:.1f} | Inference Tim: {infer_time*1000:.2f} ms/batch")
            start_time = time.time()
            frame_count = 0

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
