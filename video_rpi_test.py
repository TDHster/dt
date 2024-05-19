import cv2


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
