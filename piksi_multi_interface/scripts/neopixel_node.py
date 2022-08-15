#!/usr/bin/env python
import rospy
import board
import neopixel

def neopixel():
    pixels = neopixel.NeoPixel(board.D12, 7)
    pixels[0] = (255, 0, 0)
    pixels[1] = (0, 255, 0)
    pixels[2] = (0, 0, 255)

if __name__ == '__main__':
    try:
        neopixel()
    except rospy.ROSInterruptException:
        pass