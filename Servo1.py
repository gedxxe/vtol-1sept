import Jetson.GPIO as GPIO
import time

# Setup GPIO mode ke BOARD (berdasarkan penomoran fisik pin pada Jetson Nano)
GPIO.setmode(GPIO.BOARD)

# Pilih pin GPIO untuk mengendalikan servo
servo_pin = 12  # Gunakan pin GPIO 12 untuk kontrol servo
GPIO.setup(servo_pin, GPIO.OUT)

# Setup PWM pada pin dengan frekuensi 50Hz (umum untuk servo motor)
pwm = GPIO.PWM(servo_pin, 50)

# Mulai PWM dengan duty cycle 0 (servo tidak bergerak)
pwm.start(0)

# Fungsi untuk menggerakkan servo ke sudut tertentu (0 hingga 180 derajat)
def set_servo_angle(angle):
    # Konversi sudut (0-180 derajat) ke duty cycle PWM (rentang 2% hingga 12%)
    duty = 2 + (angle / 18)  # 2% untuk sudut 0°, 12% untuk sudut 180°
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Tunggu servo mencapai posisi
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        print("Menggerakkan servo ke 90 derajat...")
        set_servo_angle(90)  # Menggerakkan servo ke 90 derajat
        time.sleep(2)  # Tunggu selama 2 detik

        print("Menggerakkan servo kembali ke 0 derajat...")
        set_servo_angle(0)  # Menggerakkan servo kembali ke 0 derajat
        time.sleep(2)  # Tunggu selama 2 detik

except KeyboardInterrupt:
    print("Program dihentikan.")

finally:
    # Bersihkan setelah selesai
    pwm.stop()
    GPIO.cleanup()
