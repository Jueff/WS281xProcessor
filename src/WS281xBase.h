#pragma once
#include <cstdint>

class WS281xBase
{
public:
  enum WS281xColorMapping 
  {
    RGB,
    GRB
  };

  union RGBLED
  {
    uint32_t value;
    struct {
      uint8_t r;
      uint8_t g;
      uint8_t b;
    } colors;
  };

public:
  float fract(float x);
  float mix(float a, float b, float t);
  RGBLED HsvToRgb(uint8_t hue, uint8_t sat, uint8_t bright);
};