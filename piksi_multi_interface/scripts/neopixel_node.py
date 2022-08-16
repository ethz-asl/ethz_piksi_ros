#!/usr/bin/env python3
import rospy
import board
import neopixel

from libsbp_ros_msgs.msg import MsgHeartbeat

# Neopixel definitions
GPIO=board.D12
NUM_PIXELS=7
ORDER=neopixel.GRBW
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
TOGGLE_HZ=2

color_solution=RED
blinking_solution=False
has_heartbeat=False
first_pixel_rainbow=0
toggle=True

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

def heartbeat_cb(msg):
    global has_heartbeat
    if not has_heartbeat:
        color_solution = RED
        blinking_solution = False
    has_heartbeat = True
    rospy.loginfo_once("Received first heartbeat.")

def gnss_cb(msg):
    global blinking_solution, color_solution

    fix_mode = (msg.flags >> 0) & 0x7
    if fix_mode == FIX_INVALID:
        color_solution = RED
        blinking_solution = True
    elif fix_mode == FIX_SPP:
        color_solution = ORANGE
        blinking_solution = False
    elif fix_mode == FIX_DGNSS:
        color_solution = PURPLE
        blinking_solution = False
    elif fix_mode == FIX_FLOAT_RTK:
        color_solution = BLUE
        blinking_solution = True
    elif fix_mode == FIX_FIXED_RTK:
        color_solution = BLUE
        blinking_solution = False
    elif fix_mode == FIX_DEAD_RECKONING:
        color_solution = ORANGE
        blinking_solution = True
    elif fix_mode == FIX_SBAS:
        color_solution = GREEN
        blinking_solution = False
    else:
        color_solution = RED
        blinking_solution = False

def status_led():
    global first_pixel_rainbow, toggle

    rospy.init_node('status_led', anonymous=True)
    pixels.brightness=BRIGHTNESS

    rospy.Subscriber("/piksi_multi_cpp_base/base_station_receiver_0/sbp/heartbeat", MsgHeartbeat, heartbeat_cb)

    while not rospy.is_shutdown():
        if not has_heartbeat:
            rainbow(first_pixel_rainbow)
            first_pixel_rainbow = (first_pixel_rainbow + 1) % 255
            rospy.Rate(FLASH_HZ).sleep()
            rospy.logwarn_throttle(THROTTLE_PERIOD, "No Piksi heartbeat.")
        else:
            # Color ring and center according to solution 
            pixels.fill(NONE)
            for i in range(1,NUM_PIXELS):
                pixels[i] = color_solution
            if blinking_solution and toggle:
                pixels.fill(NONE)
                toggle = not toggle
            rospy.Rate(TOGGLE_HZ).sleep()


if __name__ == '__main__':
    try:
        status_led()
    except rospy.ROSInterruptException:
        pass
