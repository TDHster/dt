import cv2
import sys


def test_camera_opencv(device):
    print(f'Using video device {device}')
    # Open camera using device index 0 and explicitly request user-level access
    cap = cv2.VideoCapture(device, cv2.CAP_ANY)  # Try cv2.CAP_ANY for auto-detection
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3280)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2464)


    if not cap.isOpened():
        print("Ошибка при открытии камеры!")
        exit()

    while True:
        # Capture frame
        ret, frame = cap.read()

        if not ret:
            print("Не удалось получить кадр!")
            break

        # Process or display the frame (add your code here)
        height, width = frame.shape[:2]
        print(f'{height=}, {width=}')

        # Exit on 'q' press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()


def test_camera_picamera():
    from picamera import PiCamera

    camera = PiCamera()

    # Set resolution
    camera.resolution = (1280, 720)

    # Start a preview (press 'q' to quit)
    camera.start_preview()
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Stop preview and release resources
    camera.stop_preview()
    camera.close()


if __name__ == '__main__':

    # Get the first argument (assuming it's the filename)
    # device = sys.argv[1]
    rtsp_url = "rtsp://localhost:8554/cam"
    # device = 0
    device = rtsp_url
    test_camera_opencv(device)
