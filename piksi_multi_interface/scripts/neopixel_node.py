#!/usr/bin/env python
import rospy
import board
import neopixel

def status_led():
    pixels = neopixel.NeoPixel(board.D12, n=7, bpp=4)
    pixels[1] = (255, 0, 0)
    pixels[2] = (0, 255, 0)
    pixels[3] = (0, 0, 255)

if __name__ == '__main__':
    try:
        status_led()
    except rospy.ROSInterruptException:
        pass