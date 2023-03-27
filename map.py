import serial
import time
import pygame
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from OccupancyGrid import OccupancyGrid
from ScanMatcher_OGBased import ScanMatcher

crc_table = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
]

debug = False
pygame.init()
width = 480
size = (width, width)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Lidar mapping")
clock = pygame.time.Clock()
done = False
WHITE = (255, 255, 255)
BOT_COLOR = (0, 255, 255)

initMapXLength, initMapYLength, unitGridSize, lidarFOV, lidarMaxRange = 4 , 4, 0.032,  2 * np.pi, 12 # in Meters
wallThickness = 2 * unitGridSize
numSamplesPerRev = 450
initXY = {"x": 0.0, "y": 0.0, "theta": (9 * np.pi) / 8}
og = OccupancyGrid(initMapXLength, initMapYLength, initXY, unitGridSize, lidarFOV, numSamplesPerRev, lidarMaxRange, wallThickness)
scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid, wallThickness, moveRSigma, maxMoveDeviation, turnSigma, \
    missMatchProbAtCoarse, coarseFactor = 1.4, 0.25, 2, 5 * unitGridSize, 0.1, 0.25, 0.3, 0.15, 5
sm = ScanMatcher(og, scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid, moveRSigma, maxMoveDeviation, turnSigma, missMatchProbAtCoarse, coarseFactor)

def draw_robot():
    screen.set_at((width // 2 - 1, width // 2 - 1), BOT_COLOR)
    screen.set_at((width // 2, width // 2), BOT_COLOR)
    screen.set_at((width // 2, width // 2 - 1), BOT_COLOR)
    screen.set_at((width // 2, width // 2), BOT_COLOR)

def clear_screen():
    screen.fill((0,0,0))
    draw_robot()
    
clear_screen()

with serial.Serial("/dev/ttyUSB0", 230400, timeout=1) as serial:
    while not done:
        # Sync with header
        while True:
            b = serial.read(2)
            if not b:
                continue
            if b[0] == 0x54 and b[1] == 0x2c:
                serial.read(45)
                b = serial.read(47)
                if b[0] == 0x54 and b[1] == 0x2c:
                    break

        frame = 0
        idx = 0
        start_of_scan = False
        ranges = []
        num_scans = 0
        min_angle = 36000

        while not done:
            b = serial.read(47)
            frame += 1

            if frame == 100:
                frame = 0
                clear_screen()

            start_angle = b[5] * 256 + b[4]
            if debug:
                #print("".join("{0:02x}".format(x) for x in b))
                print("Scan number:", num_scans, end=", ")
                print("Start of scan:", start_of_scan)
                print("speed:", b[3] * 256 + b[2], end=", ")
                print("start angle:", start_angle)
            angle = start_angle
            if angle < min_angle:
                min_angle = angle
                print("Reset min angle:", min_angle)
                idx = 0
                start_of_scan = True
                ranges = []
            for i in range(12):
                distance = b[1*3 + 7] * 256 + b[i*3 + 6]
                intensity = b[i*3 + 8]
                if (i != 0):
                    angle = angle + 80
                    if idx < 449:
                        idx += 1
                        start_of_scan = False
                    else:
                        idx = 0
                        start_of_scan = True
                        num_scans += 1
                        ranges = []
                rad = math.radians(angle / 100)
                y = width // 2 - int(distance * math.sin(rad) / 16)
                x = width // 2 - int(distance * math.cos(rad) / 16)
                if debug:
                    print("Scan number:", num_scans, end=", ")
                    print("Start of scan:", start_of_scan)
                    print("Angle idx:", idx)
                    print("Radians:", rad, end = ", ")
                    print("x:", x,  end=", ")
                    print("y:", y)
                    print("Angle:", angle, end=", ")
                    print("Distance:", distance, end=", ")
                    print("Intensity:", intensity)
                screen.set_at((width - y, x), (intensity, intensity, intensity))
                ranges.append(distance / 1000)
                if num_scans > 1 and len(ranges) == 450:
                    ranges.reverse()
                    og.updateOccupancyGrid({"x":0.0, "y": 0.0, "theta": (19 * np.pi) / 16, "range":ranges})
                    og.plotOccupancyGrid()

            if debug:
                print("end angle:", b[43] * 256 + b[42], end=", ")
                print("timestamp:", b[45] * 256 + b[44], end=", ")
                print("checksum:", b[46])
            crc = 0
            for i in range(46):
                crc = crc_table[(crc ^ b[i]) & 0xff]
                
            if crc != b[46]:
                print("CRC Error at", time.time())
                break

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True

            pygame.display.flip()
            clock.tick(60)
    
    pygame.quit()

