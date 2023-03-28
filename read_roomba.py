import serial

with serial.Serial("/dev/ttyUSB0", 230400, timeout=10) as serial:
    # Sync with header
    while True:
        data = serial.read(26)
        #print(data)
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
        print("Distance:", (data[12] << 8) + data[13])
        print("Angle:", (data[14] << 8) + data[15])
        print("Charging state:", data[16])
        print("Voltage:", (data[17] << 8) + data[18])
        print("Current:", (data[19] << 8) + data[20])
        print("Temperature:", data[21])
        print("Charge:", (data[22] << 8) + data[23])
        print("Capacity:", (data[24] << 8) + data[25])




