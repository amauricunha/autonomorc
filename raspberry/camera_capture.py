import cv2
import os

def capture_camera_frames(output_dir="frames", camera_id=0):
    os.makedirs(output_dir, exist_ok=True)
    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("Erro ao abrir a câmera CSI.")
        return
    frame_count = 0
    print("Capturando frames da câmera CSI...")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Falha ao capturar frame.")
            break
        filename = f"{output_dir}/frame_{frame_count:06d}.jpg"
        cv2.imwrite(filename, frame)
        frame_count += 1
        if frame_count % 30 == 0:
            print(f"{frame_count} frames capturados.")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()

if __name__ == "__main__":
    capture_camera_frames()
