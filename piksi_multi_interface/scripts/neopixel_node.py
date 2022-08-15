#!/usr/bin/env python
import rospy
import board
import neopixel

def status_led():
    pixels = neopixel.NeoPixel(board.D12, n=7, bpp=4)
    rospy.init_node('status_led', anonymous=True)
    rate = rospy.Rate(10)
    idx = 0
    while not rospy.is_shutdown():
        idx = (idx % 7)
        pixels[idx + 1] = (255, 0, 0)
        pixels[idx + 2] = (0, 255, 0)
        pixels[idx + 3) + 1] = (0, 0, 255)
        idx = idx + 1
        rate.sleep()

if __name__ == '__main__':
    try:
        status_led()
    except rospy.ROSInterruptException:
        pass