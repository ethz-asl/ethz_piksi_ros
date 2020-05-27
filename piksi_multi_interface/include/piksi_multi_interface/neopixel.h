#ifndef PIKSI_MULTI_INTERFACE_NEOPIXEL_H_
#define PIKSI_MULTI_INTERFACE_NEOPIXEL_H_

#include <string>
#include <vector>

namespace piksi_multi_interface {

class Neopixel {
 public:
  Neopixel(const uint8_t num_pixels, const std::string& gpio_chip,
           const uint8_t gpio_line);

  void begin();
  void setPixelColor(uint8_t n, uint8_t r, uint8_t g, uint8_t b);
  void setPixelBrightness(uint16_t n, uint8_t brightness);
  void updatePixels();

 private:
  std::string gpio_chip_ = "gpiochip0";
  uint8_t gpio_line_ = 0;

  struct Pixel {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    uint8_t brightness = 0;
  };
  std::vector<Pixel> pixels_;
};

}  // namespace piksi_multi_interface

#endif  // PIKSI_MULTI_INTERFACE_NEOPIXEL_H_
