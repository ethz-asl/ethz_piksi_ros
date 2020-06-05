#include "piksi_multi_interface/neopixel.h"

#include <gpiod.h>

namespace piksi_multi_interface {

Neopixel::Neopixel(const uint8_t num_pixels, const std::string& gpio_chip,
                   const uint8_t gpio_line)
    : gpio_chip_(gpio_chip), gpio_line_(gpio_line), pixels_(num_pixels) {}

}  // namespace piksi_multi_interface
