

class Mavlink_RPi_GPIO:
    def __init__(self, GPIO_IN=[33, 34], GPIO_OUT=[36, 37]):
        try:
            import RPi.GPIO as GPIO
            print(f"Running on: {GPIO.RPI_INFO}")
        except RuntimeError:
            print("Error importing RPi.GPIO!  Are your root? Use sudo")

        try:
            #Settig up RPi board
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)

            GPIO.setup(GPIO_IN, GPIO.IN)
            GPIO.setup(GPIO_OUT, GPIO.OUT)
        except Exception as e:
            print(f'RPi pin setup error: {e}')


