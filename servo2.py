import serial
import time

# Sesuaikan port serial di bawah ini dengan port yang sesuai di Jetson
arduino = serial.Serial('/dev/ttyUSB0', 9600)  
time.sleep(2)  # Tunggu hingga koneksi terbentuk

def send_servo_angle(angle):
    if 0 <= angle <= 180:
        arduino.write(f"{angle}\n".encode())  # Mengirim data sudut ke Arduino
    else:
        print("Sudut harus antara 0 dan 180 derajat")

try:
    while True:
        angle = int(input("Masukkan sudut servo (0-180): "))
        send_servo_angle(angle)
except KeyboardInterrupt:
    print("Program dihentikan")

arduino.close()
