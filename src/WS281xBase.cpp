/*
 * SPDX-FileCopyrightText: 2025-2026 Juergen Winkler <MobaLedLib@gmx.at>
 * SPDX-License-Identifier: CC-BY-NC-4.0
 *
 * WS281xBase: Base class providing utility functions and data structures for WS281x LED color processing.
 * It includes an enum for color mapping (RGB or GRB), a union for representing RGB LED colors,
 * and static methods for mathematical operations like fract and mix, as well as HSV to RGB conversion.
*/

#include "WS281xBase.h"
#include <cmath>
#include <Arduino.h>

float WS281xBase::fract(float x)
{
  return x - int(x);
}

float WS281xBase::mix(float a, float b, float t)
{
  return a + (b - a) * t;
}

WS281xBase::RGBLED WS281xBase::HsvToRgb(uint8_t hue, uint8_t sat, uint8_t bright)
{
  RGBLED result;
  float h1 = ((float)hue) / 255;
  float s1 = ((float)sat) / 255;
  float b1 = ((float)bright) / 255;
  result.colors.r = b1 * mix(1.0, constrain(abs(fract(h1 + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s1) * 255;
  result.colors.g = b1 * mix(1.0, constrain(abs(fract(h1 + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s1) * 255;
  result.colors.b = b1 * mix(1.0, constrain(abs(fract(h1 + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s1) * 255;
  return result;
}