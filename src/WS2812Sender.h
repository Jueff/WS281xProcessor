#pragma once

#include <stdio.h>
#include <array>
#include <cmath>

#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "WS281xBase.h"

class WS2812Sender : WS281xBase
{
private:
  uint offset_sender;
  uint sm_sender;
  pio_hw_t* pio;

public:
  WS2812Sender();
  ~WS2812Sender();

  bool init(uint8_t pin);

  void setStatusLEDColor(RGBLED led);
  void setStatusLEDColor(uint8_t r, uint8_t g, uint8_t b);
  void setStatusLEDHSV(uint8_t hue, uint8_t sat, uint8_t bright);
};
