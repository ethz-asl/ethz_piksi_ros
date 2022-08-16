#!/usr/bin/env python3
import rospy
import board
import neopixel

# Neopixel definitions
GPIO=board.D12
NUM_PIXELS=7
ORDER=neopixel.RGBW
BRIGHTNESS=0.1
pixels = neopixel.NeoPixel(GPIO, n=NUM_PIXELS, pixel_order=ORDER)

NONE=(0, 0, 0, 0)
RED=(255, 0, 0, 0)
ORANGE=(255, 127, 0, 0)
YELLOW=(255, 255, 0, 0)
GREEN=(0, 255, 0, 0)
BLUE=(0, 0, 255, 0)
PURPLE=(143, 0, 255, 0)
MAGENTA=(255, 0, 255, 0)
WHITE=(255, 255, 255, 0)
FLASH_HZ=1000
THROTTLE_PERIOD=3

# State definitions
FIX_INVALID=0
FIX_SPP=1
FIX_DGNSS=2
FIX_FLOAT_RTK=3
FIX_FIXED_RTK=4
FIX_DEAD_RECKONING=5
FIX_SBAS=6
LEDS_PER_SAT=3
TOGGLE_MS=500

color_solution=RED
blinking_solution=False
has_heartbeat=False
first_pixel_rainbow=0

def wheel(pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
    if pos < 0 or pos > 255:
        r = g = b = 0
    elif pos < 85:
        r = int(pos * 3)
        g = int(255 - pos * 3)
        b = 0
    elif pos < 170:
        pos -= 85
        r = int(255 - pos * 3)
        g = 0
        b = int(pos * 3)
    else:
        pos -= 170
        r = 0
        g = int(pos * 3)
        b = int(255 - pos * 3)
    return (r, g, b) if ORDER in (neopixel.RGB, neopixel.GRB) else (r, g, b, 0)


def rainbow(first_pixel):
    for i in range(NUM_PIXELS):
        pixel_index = (i * 256 // NUM_PIXELS) + first_pixel
        pixels[i] = wheel(pixel_index & 255)

def status_led():
    rospy.init_node('status_led', anonymous=True)
    pixels.brightness=BRIGHTNESS

    while not rospy.is_shutdown() and not has_heartbeat:
        global first_pixel_rainbow
        rainbow(first_pixel_rainbow)
        first_pixel_rainbow = (first_pixel_rainbow + 1) % 255
        rospy.Rate(FLASH_HZ).sleep()
        rospy.logwarn_throttle(THROTTLE_PERIOD, "No Piksi heartbeat.")

    rospy.spin()


if __name__ == '__main__':
    try:
        status_led()
    except rospy.ROSInterruptException:
        pass
