import serial
import math

debug = False

with serial.Serial("/dev/ttyUSB0", 230400, timeout=10) as serial:
    # Set initial pose
    pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
    while True:
        data = serial.read(26)
        if not data:
            continue
        distance = (data[12] << 8) + data[13]
        if distance > 0x7fff:
            distance -= 0x10000
        pose["y"] +=  distance * math.cos(pose["theta"])
        pose["x"] += distance * math.sin(pose["theta"])
        angle = (data[14] << 8) + data[15]
        if angle > 0x7fff:
            angle -= 0x10000
        pose["theta"] += ((2 * angle) / 258)
        current = (data[19] << 8) + data[20]
        if current > 0x7fff:
            current -= 0x10000
        if debug:
            print("Bumps and wheel drops:", data[0])
            print("Wall:", data[1])
            print("Cliff left:", data[2])
            print("Cliff front left:", data[3])
            print("Cliff front right:", data[4])
            print("Cliff right:", data[5])
            print("Virtual wall:", data[6])
            print("Motor over-currents:", data[7])
            print("Dirt detector left:", data[8])
            print("Dirt detector right:", data[9])
            print("Remote command:", data[10])
            print("Buttons:", data[11])
            print("Distance:", distance)
            print("Angle:", angle)
            print("Charging state:", data[16])
            print("Voltage:", (data[17] << 8) + data[18])
            print("Current:", current)
            print("Temperature:", data[21])
            print("Charge:", (data[22] << 8) + data[23])
            print("Capacity:", (data[24] << 8) + data[25])

        temp = data[21]
        if temp < 30 or temp > 40:
            print("temperature out of range")
            break

        print("Angle:", angle)
        print("Pose:", pose)

