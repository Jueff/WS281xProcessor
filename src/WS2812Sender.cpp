/*
 * SPDX-FileCopyrightText: 2025-2026 Juergen Winkler <MobaLedLib@gmx.at>
 * SPDX-License-Identifier: CC-BY-NC-4.0
 *
 * WS2812Sender: Class for sending WS2812 LED data using PIO on RP2040 microcontrollers.
 * It initializes a PIO state machine to output LED signals on a specified pin and provides
 * methods to set the color of a status LED in RGB or HSV formats.
*/

#include "WS2812Sender.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "rp2040_pio.h"

WS2812Sender::WS2812Sender()
{
}

bool WS2812Sender::init(uint8_t pin)
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

WS2812Sender::~WS2812Sender()
{
  if (sm_sender != -1)
  {
    pio_sm_set_enabled(pio0, sm_sender, false);
    // release the program 
    pio_remove_program(pio0, &ws2812sender_program, offset_sender);
    pio_sm_unclaim(pio0, sm_sender);
  }
}

void WS2812Sender::setStatusLEDColor(RGBLED led)
{
  setStatusLEDColor(led.colors.r, led.colors.g, led.colors.b);
}

void WS2812Sender::setStatusLEDColor(uint8_t r, uint8_t g, uint8_t b)
{
  pio_sm_put(pio, sm_sender, (g << 24) | (r << 16) | (b << 8));
}

void WS2812Sender::setStatusLEDHSV(uint8_t hue, uint8_t sat, uint8_t bright)
{
  setStatusLEDColor(HsvToRgb(hue, sat, bright));
}