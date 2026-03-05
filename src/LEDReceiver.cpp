#include <Arduino.h>
#include "LEDReceiver.h"
#include "WS281xProcessor.h"

LEDReceiver::LEDReceiver(uint8_t* ledData, uint8_t numLedsToEmulate, uint8_t numLedsToSkip, uint8_t dataInPin, uint8_t dataOutPin)
    : ledData(ledData)
{
    ledDataLen = numLedsToEmulate * 3;
    ledDataPrevious = (uint8_t*)malloc(ledDataLen);
    memset(ledDataPrevious, 0, ledDataLen);

    ledDataReceived = (WS281xBase::RGBLED*)malloc(numLedsToEmulate);

    state = State::Unknown;
    defaultReconnectCycles = DEFAULT_RECONNECT_CYCLES;

    isOnline = false;
    dataChanged = false;
    errorDetected = false;
    dataAvailable = false;

    pWs281xProcessor = new WS281xProcessor();
    pWs281xProcessor->registerReceiveErrorCallback([this]() { this->WS281xProcessor_ReceiveError(); });
    pWs281xProcessor->registerDataReceivedCallback([this]() { this->WS281xProcessor_DataReceived(); });
    pWs281xProcessor->init(dataInPin, DEFAULT_SIDESET_PIN, dataOutPin, numLedsToEmulate, numLedsToSkip, WS281xBase::GRB, true);
}

LEDReceiver::~LEDReceiver()
{
    if (ledDataPrevious)
    {
        free(ledDataPrevious);
        ledDataPrevious = nullptr;
    }

    if (ledDataReceived)
    {
      free(ledDataReceived);
      ledDataReceived = nullptr;
    }

    if (pWs281xProcessor) 
    {
        delete pWs281xProcessor;
        pWs281xProcessor = nullptr;
    }
}

void LEDReceiver::setReconnectCycles(uint8_t value)
{
    defaultReconnectCycles = value;
}

LEDReceiver::State LEDReceiver::getState() const
{
    return state;
}

bool LEDReceiver::hasDataChanged()
{
    auto result = dataChanged;
    dataChanged = false;
    return result;
}

void LEDReceiver::loop()
{
    if (errorDetected) 
    {
        Serial.println("*** errorDetected ***");
        errorDetected = false;
        state = State::Error;
    }
    if ((millis() - lastDataMillis) > 500) 
    {
        auto newStatus = errorDetected ? State::Error : State::Offline;
        if (state != newStatus)
        {
            state = newStatus;
            isOnline = false;
            pWs281xProcessor->reset();
        }
    }
    else if (pWs281xProcessor->notEnoughData()) 
    {
        Serial.println("*** notEnoughData ***");
        isOnline = false;
        if (state != State::DataMissing)
        {
            state = State::DataMissing;
        }
    }
    pWs281xProcessor->setRepeaterLEDColor(ledData[0], ledData[1], ledData[2]); //TODO

    
    pWs281xProcessor->getLEDs(ledDataReceived);
    if (dataAvailable && !errorDetected) 
    {
        dataAvailable = false;
        if (!isOnline) 
        {
            isOnline = true;
            reconnectCycles = defaultReconnectCycles;
            lastDataMillis = millis() + 200;
        }
        if (millis() >= lastDataMillis) 
        {
            lastDataMillis = millis();
            for (auto cnt = 0; cnt < (uint8_t)(ledDataLen/3); cnt++)
            {
                ledData[cnt * 3] = ledDataReceived[cnt].colors.r;
                ledData[cnt * 3 + 1] = ledDataReceived[cnt].colors.g;
                ledData[cnt * 3 + 2] = ledDataReceived[cnt].colors.b;
            }
            bool dataSame = memcmp(ledData, ledDataPrevious, ledDataLen) == 0;

            if (reconnectCycles > 0) 
            {
                if (dataSame)
                {
                    reconnectCycles--;
                    dataSame = false;
                }
                else
                {
                    memcpy(ledDataPrevious, ledData, ledDataLen);
                    reconnectCycles = defaultReconnectCycles;
                }
            }
            if (reconnectCycles == 0)
            {
                if (state != State::Online) 
                {
                    state = State::Online;
                }
                if (!dataSame) 
                {
                    memcpy(ledDataPrevious, ledData, ledDataLen);
                    DebugOutputLedData();
                    dataChanged = true;
                    dataChangedMillis = millis();
                }
            }
        }
    }
}

void LEDReceiver::DebugOutputLedData()
{
    Serial.print(" LED DATA: ");
    for (int i = 0; i < ledDataLen; i++)
    {
        Serial.printf("%2x ", ledData[i]);
    }
    Serial.println();
}

void LEDReceiver::WS281xProcessor_ReceiveError()
{
    errorDetected = true;
}

void LEDReceiver::WS281xProcessor_DataReceived()
{
    dataAvailable = true;
    dataReceivedMillis = millis();
}
