import cv2

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Add cv2.CAP_V4L2 for user-level access (optional)

if not cap.isOpened():
    print("Ошибка при открытии камеры!")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Не удалось получить кадр!")
        print(f'Код ошибки: {int(ret)}')
        break

    # Отображение разрешения кадра
    height, width = frame.shape[:2]
    print(f"Разрешение кадра: {width}x{height}")

    # Отображение кадра (необязательно)
    # cv2.imshow('Камера', frame)

    # Обработка нажатия клавиши 'q' для выхода
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
