import cv2

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Ошибка при открытии камеры!")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Не удалось получить кадр!")
        break

    # Отображение разрешения кадра
    height, width = frame.shape[:2]
    print(f"Разрешение кадра: {width}x{height}")

    # Отображение кадра (необязательно)
    cv2.imshow('Камера', frame)

    # Обработка нажатия клавиши 'q' для выхода
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
