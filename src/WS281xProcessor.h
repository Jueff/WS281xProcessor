#pragma once

#include <stdio.h>
#include <array>
#include <cmath>
#include <functional>

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "WS281xBase.h"
#include "ws281xReceiver.pio.h"
#include "ws281xRepeater.pio.h"

#if defined (__cplusplus)
extern "C" {
#endif
  extern void WS281xProcessor_DataReceived() __attribute__((weak));
  extern void WS281xProcessor_ReceiveError() __attribute__((weak));
#if defined (__cplusplus)
}
#endif

#ifndef DEFAULT_SIDESET_PIN
#define DEFAULT_SIDESET_PIN 22
#endif

class WS281xProcessor : public WS281xBase
{
private:
  uint offset_receiver;
  int sm_receiver;
  pio_sm_config sm_conf;

  uint offset_repeater;
  uint sm_repeater;
  pio_sm_config sm_conf_repeater;

  uint dma_ctrl_chan;
  uint dma_gather_chan;
  dma_channel_config dma_ctrl_conf;
  dma_channel_config dma_gather_conf;

  volatile uint32_t* led_state_address;
  uint8_t ledsToSkip;
  uint8_t ledsToRead;
  WS281xColorMapping color_mapping;
  bool statusLEDActive;

  std::function<void()> dataReceivedCallback;
  std::function<void()> receiveErrorCallback;

  inline void init_receiverGPIO(uint8_t dataInPin, uint8_t sidesetPin);
  inline void initSMConfig(uint8_t dataInPin, uint8_t sidesetPin, uint8_t dataOutPin);
  inline void initDMA();
  inline void runSM(uint8_t dataInPin);
  inline void initGPIO_repeater(uint8_t dataInPin, uint8_t dataOutPin);
  inline void initSMConfig_repeater(uint8_t dataInPin, uint8_t sidesetPin, uint8_t dataOutPin);
  inline void runRepeaterSM();
  inline void stopRepeaterSM();

public:
  WS281xProcessor();
  ~WS281xProcessor();

  void init(uint8_t dataInPin, uint8_t sidesetPin, uint8_t dataOutPin, uint8_t ledsToRead, uint8_t ledsToSkip, WS281xColorMapping mapping, bool statusLEDActive);
  const RGBLED getLED(uint8_t idx) const;
  const void startGatherDma();
  inline const RGBLED ledStateToLED(const uint32_t val) const;
  void registerDataReceivedCallback(std::function<void()> cb);
  void registerReceiveErrorCallback(std::function<void()> cb);
  void notifyDataReceived();
  void notifyReceiveError();
  static constexpr const uint shift(const uint val);
  static constexpr std::pair<uint, uint> getBitOffsets(uint numLeds);
  const void getLEDs(RGBLED* leds);
  const uint getLedsToRead() const;
  const int getResetCnt() const;
  bool notEnoughData();
  void setRepeaterLEDColor(RGBLED led);
  void setRepeaterLEDHSV(uint8_t hue, uint8_t sat, uint8_t bright);
  void setRepeaterLEDColor(uint8_t r, uint8_t g, uint8_t b);
  void resetRxBuffer();
  void reset();
};

// IRQ Handler Deklarationen
void ws281xRepeater_pio_irq0_handler();
void ws281xReceiver_pio1_irq0_handler();

