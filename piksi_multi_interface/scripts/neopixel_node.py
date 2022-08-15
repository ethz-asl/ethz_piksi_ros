#!/usr/bin/env python
import rospy
import board
import neopixel

def status_led():
    pixels = neopixel.NeoPixel(board.D12, n=7, bpp=4)
    rate = rospy.Rate(10)
    idx = 1
    while not rospy.is_shutdown():
        pixels[(idx % 7) + 1] = (255, 0, 0)
        pixels[(idx + 1 % 7) + 1] = (0, 255, 0)
        pixels[(idx + 2 % 7) + 1] = (0, 0, 255)
        idx = idx + 1
        rate.sleep()

if __name__ == '__main__':
    try:
        status_led()
    except rospy.ROSInterruptException:
        pass