#pragma once

#include "WS281xBase.h"
class WS281xProcessor;

class LEDReceiver
{
public:
  enum class State
  {
    Unknown = -1,
    Error = 0,
    DataMissing = 1,
    Offline = 2,
    Online = 3
  };

  LEDReceiver(uint8_t* ledData, uint8_t ledsToRead, uint8_t ledsToSkip, uint8_t dataInPin, uint8_t dataOutPin);
  LEDReceiver(uint8_t* ledData, uint8_t ledsToRead, uint8_t ledsToSkip, uint8_t dataInPin, uint8_t dataOutPin, bool statusLEDActive);
  ~LEDReceiver();

  void setReconnectCycles(uint8_t value);
  void setNoDataTimeout(uint value);
  State getState() const;
  bool hasDataChanged();
  void loop();
  void setRepeaterLEDHSV(uint8_t hue, uint8_t sat, uint8_t bright);
  void setRepeaterLEDColor(uint8_t r, uint8_t g, uint8_t b);

  void DebugOutputLedData();
  void WS281xProcessor_ReceiveError();
  void WS281xProcessor_DataReceived();

private:
  static constexpr uint DEFAULT_RECONNECT_CYCLES = 2;
  static constexpr uint DEFAULT_NODATA_TIMEOUT   = 1000;

  WS281xProcessor* pWs281xProcessor;

  uint8_t* ledData;
  uint8_t* ledDataPrevious;
  uint8_t* ledDataBuffer;
  size_t ledDataLen;
  WS281xBase::RGBLED* ledDataReceived;

  uint8_t defaultReconnectCycles;
  uint    noDataTimeout;

  // State variables
  State state;
  bool isOnline;
  bool dataChanged;
  bool dataAvailable;
  bool errorDetected;
  uint8_t reconnectCycles;
  unsigned long lastDataMillis;
  unsigned long dataReceivedMillis;
  unsigned long dataChangedMillis;
};
