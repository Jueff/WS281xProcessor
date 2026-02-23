const uint DATA_IN_PIN = 27;
const uint DATA_OUT_PIN = 26;
const uint DATA_IN_PIN2 = 27;
const uint STATUSLED_PIN = 16;

const uint NUM_LEDS_TO_EMULATE = 3;
const uint NUM_LEDS_TO_SKIP = 2; // 5 or 17

const uint DEFAULT_RECONNECT_CYCLES = 2;

#include "WS281xProcessor.h"

WS2811Processor* pWs2811Processor = NULL;
int lastSec = millis();
int sampleMs = 0;

bool isOnline = false;
uint8_t reconnectCycles = DEFAULT_RECONNECT_CYCLES;

volatile bool dataAvailable = false;
volatile bool errorDetected  = false;

volatile uint16_t systemTimeInTicks=0;

uint8_t ledData[NUM_LEDS_TO_EMULATE*3];
uint8_t ledDataPrevious[NUM_LEDS_TO_EMULATE*3];

void setup()
{
    // only enable for debgging purpose to see trace output of boot code
    //while (!tud_cdc_connected()) {} 
    Serial.begin(115200);
    Serial.println("Receive Two LEDs");

    pWs2811Processor = new WS2811Processor();
    pWs2811Processor->init(NUM_LEDS_TO_EMULATE, NUM_LEDS_TO_SKIP, GRB, true);
}

void loop()
{
  auto lastDataMillis = millis() + 1000;   // at start wait 1secs
  uint8_t cnt=0;
  int status = -1;
  while (true) 
  {
    if (errorDetected)
    {
      Serial.println("*** errorDetected ***");
      errorDetected = false;
    }
    if ((millis()-lastDataMillis)>50)
    {
      status = 1;
      isOnline = false;
    }
    else if (pWs2811Processor->notEnoughData())   // highest priority for status signaling
    {
      Serial.println("*** notEnoughData ***");
      isOnline = false;
      status = 0;
    }

    std::array<RGBLED,NUM_LEDS_TO_EMULATE> leds;
    pWs2811Processor->getLEDs(&leds[0]);
    if (dataAvailable && ! errorDetected)
    {
      dataAvailable = false;
      if (!isOnline) 
      {
        isOnline = true;
        reconnectCycles = DEFAULT_RECONNECT_CYCLES;
        lastDataMillis = millis() + 200;   // after reconnect wait 200msecs for next data
      }
      if (millis()>=lastDataMillis) 
      {
          lastDataMillis = millis();
          for (auto it = leds.begin(); it != leds.end(); it++) {
            ledData[cnt*3] = (*it).colors.r;
            ledData[cnt*3+1] = (*it).colors.g;
            ledData[cnt*3+2] = (*it).colors.b;
            cnt++;
          }
          bool dataSame = memcmp(ledData, ledDataPrevious, sizeof(ledData))==0;
          
          if (reconnectCycles>0)
          {
              if (dataSame) 
              {
                  reconnectCycles--;
              }
              else
              {
                  memcpy(ledDataPrevious, ledData, sizeof(ledData));
                  reconnectCycles = DEFAULT_RECONNECT_CYCLES;       // data changed, start triggering again
              }
          }
          if (reconnectCycles==0)
          {
              // connected, getting valid data
              if (status!=2)
              {
                Serial.println("connected");
                status=2;
              }
              if (!dataSame)
              {
                memcpy(ledDataPrevious, ledData, sizeof(ledData));
                for (uint i=0;i<NUM_LEDS_TO_EMULATE*3;i++)
                {
                  Serial.printf("%2x ", ledData[i]);
                }
                Serial.println();
              }
          }
       }
    }
    if ((millis()-lastSec)>=1000)
    {
      lastSec+=1000;
      for (uint i=0;i<NUM_LEDS_TO_EMULATE*3;i++)
      {
        Serial.printf("%2x ", ledData[i]);
      }
      Serial.println();
    }
  }
}

void WS2811Processor_ReceiveError()
{
  errorDetected = true;
}

void WS2811Processor_DataReceived()
{
  dataAvailable = true;
}