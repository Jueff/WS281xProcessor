#pragma once

#include <stdio.h>
#include <array>
#include <cmath>

#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "ws281xCommon.h"
#include "rp2040_pio.h"

class WS2812Sender
{
private:
  uint offset_sender;
  uint sm_sender;
  pio_hw_t* pio;

public:

  WS2812Sender()
  {
  }

  bool init(uint8_t pin)
  {
    for (int i = 0; i < 2; i++)
    {
      pio = i == 0 ? pio0 : pio1;
      sm_sender = pio_claim_unused_sm(pio, false);
      if (sm_sender >= 0)
      {
        offset_sender = pio_add_program(pio, &ws2812sender_program);
        ws2812sender_program_init(pio, sm_sender, offset_sender, pin, 800000, 24);
        return true;
      }
    }
    sm_sender = -1;
    return false;
  }

  ~WS2812Sender()
  {
    if (sm_sender != -1)
    {
      pio_sm_set_enabled(pio0, sm_sender, false);
      // release the program 
      pio_remove_program(pio0, &ws2812sender_program, offset_sender);
      pio_sm_unclaim(pio0, sm_sender);
    }
  }

  void setStatusLEDColor(RGBLED led)
  {
    setStatusLEDColor(led.colors.r, led.colors.g, led.colors.b);
  }

  void setStatusLEDColor(uint8_t r, uint8_t g, uint8_t b)
  {
    pio_sm_put(pio, sm_sender, (g << 24) | (r << 16) | (b << 8));
  }

  void setStatusLEDHSV(uint8_t hue, uint8_t sat, uint8_t bright)
  {
    float h1 = ((float)hue) / 255;
    float s1 = ((float)sat) / 255;
    float b1 = ((float)bright) / 255;
    float r = b1 * mix(1.0, constrain(abs(fract(h1 + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s1);
    float g = b1 * mix(1.0, constrain(abs(fract(h1 + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s1);
    float b = b1 * mix(1.0, constrain(abs(fract(h1 + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s1);
    setStatusLEDColor(r * 255, g * 255, b * 255);
  }
};
