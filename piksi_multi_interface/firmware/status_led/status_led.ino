#define USE_USBCON

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include <ros.h>

#include <libsbp_ros_msgs/MsgAgeCorrections.h>
#include <libsbp_ros_msgs/MsgPosEcef.h>
#include <piksi_rtk_msgs/PositionSampling.h>

// Defintiions
#define PIN 6
#define NUMPIXELS 7

// Save Arduino memory.
#define MAX_SUBSCRIBERS 3
#define MAX_PUBLISHERS 0
#define INPUT_SIZE 256
#define OUTPUT_SIZE 256

// NeoPixel
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define NONE pixels.Color(0, 0, 0)
#define RED pixels.Color(255, 0, 0)
#define ORANGE pixels.Color(255, 127, 0)
#define YELLOW pixels.Color(255, 255, 0)
#define GREEN pixels.Color(0, 255, 0)
#define BLUE pixels.Color(0, 0, 255)
#define PURPLE pixels.Color(143, 0, 255)
#define MAGENTA pixels.Color(255, 0, 255)
#define FLASH 50  // Flash time ms.

// State
#define FIX_INVALID 0
#define FIX_SPP 1
#define FIX_DGNSS 2
#define FIX_FLOAT_RTK 3
#define FIX_FIXED_RTK 4
#define FIX_DEAD_RECKONING 5
#define FIX_SBAS 6
uint32_t color_solution = RED;
bool blinking_solution = false;

#define TOGGLE_MS 500
#define LEDS_PER_SAT 3

// ROS
ros::NodeHandle_<ArduinoHardware, MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE,
                 OUTPUT_SIZE>
    nh;

// The position solution gives feedback about the current fix and the number of
// satellites in the fix.
void fixCb(const libsbp_ros_msgs::MsgPosEcef& msg) {
  uint8_t fix_mode = (msg.flags >> 0) & 0x7;
  switch (fix_mode) {
    case FIX_INVALID:
      color_solution = RED;
      blinking_solution = true;
      break;
    case FIX_SPP:
      color_solution = ORANGE;
      blinking_solution = false;
      break;
    case FIX_DGNSS:
      color_solution = PURPLE;
      blinking_solution = false;
      break;
    case FIX_FLOAT_RTK:
      color_solution = BLUE;
      blinking_solution = true;
      break;
    case FIX_FIXED_RTK:
      color_solution = BLUE;
      blinking_solution = false;
      break;
    case FIX_DEAD_RECKONING:
      color_solution = ORANGE;
      blinking_solution = true;
      break;
    case FIX_SBAS:
      color_solution = GREEN;
      blinking_solution = false;
      break;
    default:
      color_solution = RED;
      break;
  }

  // Color LEDs according to number of satellites. If error turn on all.
  for (int i = 1; i < 7; i++) {
    if (color_solution == RED || msg.n_sats >= i * LEDS_PER_SAT) {
      pixels.setPixelColor(i, color_solution);
    } else {
      pixels.setPixelColor(i, NONE);
    }
  }

  // Toggle blinking LEDs.
  auto now = millis();
  static auto last_toggle = now;
  bool toggle = now - last_toggle > TOGGLE_MS;
  if (toggle && blinking_solution) {
    for (int i = 1; i < 7; i++) {
      if (pixels.getPixelColor(i)) {
        pixels.setPixelColor(i, NONE);
      }
    }
  }
  if (now - last_toggle > 2 * TOGGLE_MS) last_toggle = now;
  pixels.show();
}

ros::Subscriber<libsbp_ros_msgs::MsgPosEcef> fix(
    "/piksi_multi_cpp_base/base_station_receiver_0/sbp/pos_ecef", fixCb);

// Flash red center LED if corrections occur.
uint16_t age = 0xFFFF;
uint32_t prev_color = NONE;
void corrCb(const libsbp_ros_msgs::MsgAgeCorrections& msg) {
  if (msg.age < age) {
    prev_color = pixels.getPixelColor(0);
    pixels.setPixelColor(0, RED);
  } else {
  //  pixels.setPixelColor(0, prev_color);
  }

  age = msg.age;
  pixels.show();
}

ros::Subscriber<libsbp_ros_msgs::MsgAgeCorrections> corr(
    "/piksi_multi_cpp_base/base_station_receiver_0/sbp/age_corrections",
    corrCb);

// Blink center LED while sampling.
void sampleCb(const piksi_rtk_msgs::PositionSampling& msg) {
  uint32_t toggle_ms = 500 * 50 / (msg.progress > 0 ? msg.progress : 1);

  auto now = millis();
  static auto last_sample = now;
  auto passed = now - last_sample;
  if (passed > toggle_ms) {
    pixels.setPixelColor(0, MAGENTA);
  } else {
    pixels.setPixelColor(0, NONE);
  }
  if (passed > 2 * toggle_ms) last_sample = now;
  pixels.show();
}

ros::Subscriber<piksi_rtk_msgs::PositionSampling> sample(
    "/piksi_multi_cpp_base/base_station_receiver_0/position_sampler/"
    "position_sampling",
    sampleCb);

// Rainbow cycle along whole strip.
long first_pixel_hue = 0;
void rainbow() {
  for (int i = 0; i < pixels.numPixels(); i++) {
    int pixelHue = first_pixel_hue + (i * 65536L / pixels.numPixels());
    pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(pixelHue)));
  }

  first_pixel_hue += 256;
  if (first_pixel_hue >= 5 * 65536) {
    first_pixel_hue = 0;
  }
  pixels.show();
}

void setup() {
  // NeoPixel
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif

  pixels.begin();

  // ROS
  nh.initNode();
  nh.subscribe(fix);
  nh.subscribe(corr);
  nh.subscribe(sample);
}

bool clear = false;
void loop() {
  if (!nh.connected()) {
    rainbow();
    delay(FLASH);
    nh.loginfo("Not connected.");
    clear = true;
  } else if (clear) {
    pixels.clear();
    clear = false;
  }
  nh.spinOnce();
}
