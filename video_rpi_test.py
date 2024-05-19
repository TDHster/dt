import cv2


def test_camera_via_opencv():
    # Open camera using device index 0 and explicitly request user-level access
    cap = cv2.VideoCapture(0, cv2.CAP_ANY)  # Try cv2.CAP_ANY for auto-detection
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2464)


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
    test_camera_picamera()
