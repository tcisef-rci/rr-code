# Title:  Rescue Rangers Test Code - Face Detection using Thermal Camera Lepton 3.5 module on OpenMV camera board (for testing only)
# Author: Richard Ilioukevitch
# Date:   1 March 2024
# Dependances: The code relies on OpenMV FLIR Lepton 3.5 and RPC libraries that comes with the recent Firmware updates (after 2020)
# and has been adopted from their example codes
# the code file should be place in the root directory of the OpenMV flash when connected to the computer
#
# NOTE: the code file name should be main.py to invoke it at start up
#
# The following code runs on OpenMV H7 camera board with FLIR Lepton 3.5 Thermal sensor installed and
# attached LCD display after powering it up
# OpenMV board is powered by external source (5v battery pack)
# OpenMV micropython code will output the result of face detection (bounding box) from camera thermal vision stream to the attached LCD display

import sensor
import time
import image
import display

# Reset sensor
sensor.reset()

# Sensor settings
sensor.set_contrast(3)
sensor.set_gainceiling(16)
# HQVGA and GRAYSCALE are the best for face tracking.
sensor.set_framesize(sensor.HQVGA)
sensor.set_pixformat(sensor.GRAYSCALE)

# Load Haar Cascade
# By default this will use all stages, lower satges is faster but less accurate.
face_cascade = image.HaarCascade("frontalface", stages=25)
print(face_cascade)

# FPS clock
clock = time.clock()
lcd = display.SPIDisplay()

while True:
    clock.tick()

    # Capture snapshot
    img = sensor.snapshot()

    # Find objects.
    # Note: Lower scale factor scales-down the image more and detects smaller objects.
    # Higher threshold results in a higher detection rate, with more false positives.
    objects = img.find_features(face_cascade, threshold=0.75, scale_factor=1.25)

    # Draw objects
    for r in objects:
        img.draw_rectangle(r)

    # Print FPS.
    # Note: Actual FPS is higher, streaming the FB makes it slower.
    lcd.write(img)
    print(clock.fps())