#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
import serial

dd = 18
baud = 115200
speed = 200

# set up the serial connection to the Roomba
ser = serial.Serial('/dev/serial0', baud, timeout=1)

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

def updatesensors():
  ser.write(bytes([142, 1]))
  time.sleep(0.5)
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
GPIO.output(dd, GPIO.HIGH)
time.sleep(0.1)
GPIO.output(dd, GPIO.LOW)
time.sleep(3)

# Start the interface
start()
control()
time.sleep(0.5)

for i in range(2):
  # drive forward at a speed of 200 mm/s
  forward()

  # wait for the Roomba to drive for a short period of time
  time.sleep(5)

  # stop the Roomba
  stop()

  spinleft()
  time.sleep(2.3)
  stop()

# close the serial connection
ser.close()
