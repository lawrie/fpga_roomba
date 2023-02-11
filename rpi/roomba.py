#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
import serial

dd = 18
baud = 115200
speed = 200
turn_time = 2.3

led_bits = 0x01
power_color = 255
power_intensity = 128

song_bytes = [67, 16, 67, 16, 67, 16, 64, 64, 66, 16, 66, 16, 66, 16, 63, 64]

def baudrate(hz):
  ser.write(bytes([129, hz]))

def forward():
  ser.write(bytes([137, (speed >> 8) & 0xff, speed & 0xff, 0x80, 0]))

def backward():
  ser.write(bytes([137,(-speed >> 8) & 0xff, -speed & 0xff, 0x80, 0]))

def arc(r):
  ser.write(bytes([137, speed >> 8 & 0xff, speed & 0xff, radius >> 8 & 0xff,  + radius & 0xff]))

def spinleft():
  ser.write(bytes([137, speed >> 8 & 0xff, speed & 0xff, 0, 1]))

def spinright():
  ser.write(bytes([137, speed >> 8 & 0xff, speed & 0xff, 0xff, 0xff]))

def motors(m=7):
  ser.write(bytes([138, m]))

def dock():
  ser.write(bytes([143]))

def control():
  ser.write(bytes([130]))

def start():
  ser.write(bytes([128]))

def sleep():
  ser.write(bytes([133]))

def safe():
  ser.write(bytes([131]))

def full():
  ser.write(bytes([132]))

def spot():
  ser.write(bytes([134]))

def max():
  ser.write(bytes([136]))

def stop():
  ser.write(bytes([137, 0, 0, 128, 0]))

def clean():
  ser.write(chr(135))

def set_leds():
  ser.write(bytes([139, led_bits, power_color, power_intensity]))

def song(n):
  ser.write(bytes([140, n] + [len(song_bytes) // 2] +   song_bytes))

def play(n):
  ser.write(bytes([141] + [n]))

def updatesensors():
  ser.write(bytes([142, 1]))
  time.sleep(0.1)
  sensors = ser.read(10)
  while len(sensors) == 10:
    temp = ser.read(10)
    if len(temp) == 0:
      break
    sensors = temp
  return sensors

def step():
  sensors = updatesensors()
  if len(sensors) == 10:
    if ord(sensors[0]) & 0x01:
      spinleft()
      motors()
      time.sleep(1)
    elif ord(sensors[0]) & 0x02:
      spinright()
      time.sleep(1)
    else:
      forward()
  else:
     print('No sensor value ',len(sensors))

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(dd, GPIO.OUT)

# Wake up roomba
GPIO.output(dd, GPIO.LOW)
time.sleep(0.1)
GPIO.output(dd, GPIO.HIGH)
time.sleep(2)

# set up the serial connection to the Roomba
ser = serial.Serial('/dev/serial0', baud, timeout=1)

# Start the interface, and switch to full mode
start()
control()
full()
time.sleep(0.25)

# Define a song
song(0)

# Drive forward and turn, twice
for i in range(2):
  # drive forward at a speed of 200 mm/s
  forward()

  # Read sensors while driving forward
  for j in range(5):
    sensors = updatesensors()
    # Play a sound if an obstacle is hit
    if (sensors[0] != 0):
      print("Sensors: ", sensors[0])
      play(0)

  # Stop the Roomba
  stop()

  # Turn 180 degrees
  spinleft()
  time.sleep(turn_time)
  stop()

# Power down
sleep()

# close the serial connection
ser.close()
