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

  LEDReceiver(uint8_t* ledData, uint8_t numLedsToEmulate, uint8_t numLedsToSkip, uint8_t dataInPin, uint8_t dataOutPin);
  ~LEDReceiver();

  void setReconnectCycles(uint8_t value);
  State getState() const;
  bool hasDataChanged();
  void loop();
  void DebugOutputLedData();
  void WS281xProcessor_ReceiveError();
  void WS281xProcessor_DataReceived();

private:
  static constexpr uint DEFAULT_RECONNECT_CYCLES = 2;

  WS281xProcessor* pWs281xProcessor;

  uint8_t* ledData;
  uint8_t* ledDataPrevious;
  uint8_t* ledDataBuffer;
  size_t ledDataLen;
  WS281xBase::RGBLED* ledDataReceived;

  uint8_t defaultReconnectCycles;

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
