#!/usr/bin/env python3

# Import Libraries
from ultralytics import YOLO
import cv2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Input Acquisition
def init_camera(width=640, height=480, index=6):
    print(f"Mencoba membuka kamera pada index {index}...")
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        print("Gagal membuka kamera")
        return None
    return cap

# Main Looping Function
def process_video(cap, model, node, bbox_pub, center_pub, fps_pub):
    while rclpy.ok():
        start_time = time.time()
        ret, frame = cap.read()
        if not ret:
            break

        # Pre-processing
        frame_resized = cv2.resize(frame, (640, 480))
        luas_h = 0
        center_hx = center_hy = 0

        # Object Prediction by Class
        results = model(frame_resized, stream=True, verbose=False, device='0')

        for result in results:
            boxes = result.boxes
            if boxes is not None and boxes.xyxy is not None:
                for i in range(len(boxes)):
                    xyxy = boxes.xyxy[i]
                    cls_id = int(boxes.cls[i].item())
                    scale_x = frame.shape[1] / frame_resized.shape[1]
                    scale_y = frame.shape[0] / frame_resized.shape[0]
                    x1 = int(xyxy[0].item() * scale_x)
                    y1 = int(xyxy[1].item() * scale_y)
                    x2 = int(xyxy[2].item() * scale_x)
                    y2 = int(xyxy[3].item() * scale_y)

                    if cls_id == 0:
                        center_hx = (x1 + x2) // 2
                        center_hy = (y1 + y2) // 2
                        luas_h = (x2 - x1) * (y2 - y1)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(frame, (center_hx, center_hy), 4, (0, 255, 0), -1)
                        break

        # Post-processing
        fps = int(1 / (time.time() - start_time))

        # Publish each value using standard Int32 messages
        from std_msgs.msg import Int32
        bbox_msg = Int32()
        bbox_msg.data = luas_h
        bbox_pub.publish(bbox_msg)

        center_msg = Int32()
        center_msg.data = center_hx
        center_pub.publish(center_msg)

        fps_msg = Int32()
        fps_msg.data = fps
        fps_pub.publish(fps_msg)

        # Visualization
        cv2.putText(frame, f"FPS: {fps}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Kamera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Call Function
def main():
    rclpy.init()
    node = rclpy.create_node('vision_pub')

    # Define 3 publishers using std_msgs/Int32
    bbox_pub = node.create_publisher(Int32, 'bounding_box_topic', 10)
    center_pub = node.create_publisher(Int32, 'center_topic', 10)
    fps_pub = node.create_publisher(Int32, 'fps_topic', 10)

    model = YOLO("yolo11n.pt")  # Ganti dengan path model
    camera = init_camera(index=6)

    if camera:
        try:
            process_video(camera, model, node, bbox_pub, center_pub, fps_pub)
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
