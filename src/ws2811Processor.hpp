#pragma once

#include <stdio.h>
#include <array>
#include <cmath>

#include "hardware/pio.h"
#include "hardware/clocks.h"

#include "ws2811Receiver.pio.h"
#include "ws2811Repeater.pio.h"
#include "rp2040_pio.h"

#if defined (__cplusplus)
extern "C" {
#endif
	extern void WS2811Processor_DataReceived() __attribute__((weak));
	extern void WS2811Processor_ReceiveError() __attribute__((weak));
#if defined (__cplusplus)
}
#endif

#ifndef SIDESET_PIN
  #define SIDESET_PIN 22
#endif

enum WS2811ColorMapping {
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

void ws2811Receiver_pio1_irq0_handler(); // forward declaration
void ws2811Repeater_pio_irq0_handler();  // forward declaration
void* ws2811Processor_instance = NULL;

ulong resetCnt = 0;
static uint32_t ws2811Repeater_repeater_led_val = 0;
static uint32_t ws2811Repeater_bits_to_write = 24;

class WS2811Processor
{

private:
  uint offset_receiver;
  int sm_receiver;
  pio_sm_config sm_conf;

  uint offset_repeater;
  uint sm_repeater;
  pio_sm_config sm_conf_repeater;

  uint offset_sender;
  uint sm_sender;
  pio_sm_config sm_conf_sender;

  uint dma_ctrl_chan;
  uint dma_gather_chan;
  dma_channel_config dma_ctrl_conf;
  dma_channel_config dma_gather_conf;

  volatile uint32_t *led_state_address = NULL;
  uint8_t ledsToSkip = 0;
  uint8_t ledsToRead = 0;
  WS2811ColorMapping color_mapping;
  bool statusLEDActive;

  inline void init_receiverGPIO(uint8_t dataInPin, uint8_t sidesetPin) {
    pio_sm_set_consecutive_pindirs(pio1, sm_receiver, dataInPin, 1, GPIO_IN);
    pinMode(dataInPin, INPUT_PULLDOWN);
    pio_gpio_init(pio1, dataInPin);
    pio_sm_set_consecutive_pindirs(pio1, sm_receiver, sidesetPin, 1, GPIO_OUT);
    pio_gpio_init(pio1, sidesetPin);
  }

  inline void initSMConfig(uint8_t dataInPin, uint8_t sidesetPin, uint8_t dataOutPin) {
    sm_conf = ws2811Receiver_program_get_default_config(offset_receiver);

    sm_config_set_in_pins(&sm_conf, dataInPin);
    
    sm_config_set_jmp_pin(&sm_conf, dataInPin);
    sm_config_set_sideset_pins(&sm_conf, sidesetPin);

    sm_config_set_in_shift(&sm_conf, false, true, 24);  // shift left, auto push after 24 bit
    sm_config_set_out_shift(&sm_conf, false, false, 0); // shift left, no auto pull
    sm_config_set_fifo_join(&sm_conf, PIO_FIFO_JOIN_RX);

    sm_config_set_clkdiv(&sm_conf, 1);
  }

  inline void initDMA() {
    dma_ctrl_conf = dma_channel_get_default_config(dma_ctrl_chan);
    dma_gather_conf = dma_channel_get_default_config(dma_gather_chan);

    // CTRL
    {
      channel_config_set_transfer_data_size(&dma_ctrl_conf, DMA_SIZE_32);
      channel_config_set_read_increment(&dma_ctrl_conf, false);
      channel_config_set_write_increment(&dma_ctrl_conf, false);
      dma_channel_configure(
        dma_ctrl_chan,
        &dma_ctrl_conf,
        &dma_hw->ch[dma_gather_chan].al2_write_addr_trig, // write
        &led_state_address,                               // read
        1,
        false
      );
    }

    // GATHER
    {
      channel_config_set_transfer_data_size(&dma_gather_conf, DMA_SIZE_32);
      channel_config_set_read_increment(&dma_gather_conf, false);
      channel_config_set_write_increment(&dma_gather_conf, true);
      channel_config_set_dreq(&dma_gather_conf, pio_get_dreq(pio1, sm_receiver, false));
      channel_config_set_chain_to(&dma_gather_conf, dma_ctrl_chan);
      dma_channel_configure(
        dma_gather_chan,
        &dma_gather_conf,
        led_state_address, // write
        &pio1->rxf[sm_receiver], // read
        this->ledsToRead,
        false
      );
    }

    dma_channel_start(dma_ctrl_chan);
  }

  inline void runSM(uint8_t dataInPin) 
  {
    pio_sm_clear_fifos(pio1, sm_receiver);
    pio_sm_init(pio1, sm_receiver, offset_receiver, &sm_conf);

    // init
    pio_sm_exec_wait_blocking(pio1, sm_receiver, pio_encode_set(pio_y, 20));
    pio_sm_exec_wait_blocking(pio1, sm_receiver, pio_encode_mov(pio_osr, pio_y));
    pio_sm_exec_wait_blocking(pio1, sm_receiver, pio_encode_out(pio_null, 5));

    // wait for first reset pulse
    bool reset_finished = false;
    while (!reset_finished) {
      while (gpio_get(dataInPin))
        ;

      const auto us = time_us_32();
      reset_finished = true;
      while (time_us_32() - us < 10) {
        if (gpio_get(dataInPin)) {
          reset_finished = false;
          break;
        }
        tight_loop_contents();
      }
    }
    pio_sm_set_enabled(pio1, sm_receiver, true);
  }

  inline void initGPIO_repeater(uint8_t dataInPin, uint8_t dataOutPin)
  {
    pio_sm_set_consecutive_pindirs(pio0, sm_repeater, dataInPin, 1, GPIO_IN);
    pio_gpio_init(pio0, dataInPin);

    pio_sm_set_consecutive_pindirs(pio0, sm_repeater, dataOutPin, 1, GPIO_OUT);
    pio_gpio_init(pio0, dataOutPin);
  }

  inline void initSMConfig_repeater(uint8_t dataInPin, uint8_t sidesetPin, uint8_t dataOutPin)
  {
    sm_conf_repeater = ws2811Repeater_program_get_default_config(offset_repeater);

    sm_config_set_in_pins(&sm_conf_repeater, dataInPin);
    sm_config_set_jmp_pin(&sm_conf_repeater, dataInPin);
    sm_config_set_out_pins(&sm_conf_repeater, dataOutPin, 1);
    sm_config_set_set_pins(&sm_conf_repeater, dataOutPin, 1);
    sm_config_set_sideset_pins(&sm_conf_repeater, sidesetPin);

    sm_config_set_in_shift(&sm_conf_repeater, false, false, 0);  // shift left, no auto push
    sm_config_set_out_shift(&sm_conf_repeater, false, true, 32); // shift left, no auto pull
    sm_config_set_fifo_join(&sm_conf_repeater, PIO_FIFO_JOIN_NONE);

    // divide by 2 to reach delays in one PIO instruction
    sm_config_set_clkdiv(&sm_conf_repeater, 2);

    irq_set_exclusive_handler(PIO0_IRQ_0, ws2811Repeater_pio_irq0_handler);

    // Enable FIFO interrupt in the PIO itself
    pio_set_irq0_source_enabled(pio0,  pis_sm0_tx_fifo_not_full, true);
    pio_set_irq0_source_enabled(pio0,  pis_sm0_rx_fifo_not_empty , true);
  }

  inline void runRepeaterSM() {
    pio_sm_clear_fifos(pio0, sm_repeater);
    pio_sm_init(pio0, sm_repeater, offset_repeater, &sm_conf_repeater);

    // init

    // move number of bits to skip into output shift register
    pio_sm_put(pio0, 0, ledsToSkip*24);

    // move the OSR to ISR - ISR holds the bits to skip in pio program
    pio_sm_exec_wait_blocking(pio0, sm_repeater, pio_encode_pull(false, false));
    pio_sm_exec_wait_blocking(pio0, sm_repeater, pio_encode_mov(pio_x, pio_osr));
    pio_sm_exec_wait_blocking(pio0, sm_repeater, pio_encode_mov(pio_isr, pio_x));

    // Enable IRQ in the NVIC
    irq_set_enabled(PIO0_IRQ_0, true);

    pio_sm_set_enabled(pio0, sm_repeater, true);
  }

  inline void stopRepeaterSM() {
    // Disable IRQ in the NVIC
    irq_set_enabled(PIO0_IRQ_0, false);
    pio_sm_set_enabled(pio0, sm_repeater, false);
  }

public:
  WS2811Processor()
  {
  }

  void init(uint8_t dataInPin, uint8_t sidesetPin, uint8_t dataOutPin, uint8_t ledsToRead, uint8_t ledsToSkip, WS2811ColorMapping mapping, bool statusLEDActive)
  {
    if (this->ledsToSkip!=0 || this->ledsToRead!=0)
    {
      // already init;
      panic("Cannot init WS2811 processor twice!");

      // maybe support re-init in the future
      //if (this->led_state_address!=null) free(this->led_state_address);
      return;
    }
    this->ledsToSkip = ledsToSkip;
    this->ledsToRead = ledsToRead;
    this->statusLEDActive = statusLEDActive;
    this->led_state_address = (uint32_t*)malloc(ledsToRead*sizeof(uint32_t));
    this->color_mapping = mapping;
    ws2811Processor_instance = this;

    if (!pio_can_add_program(pio1, &ws2811Receiver_program)) {
      panic("Cannot start WS2811 client because PIOs do not have enough space.");
    }

    if (!pio_can_add_program(pio0, &ws2811Repeater_program)) 
    {
      panic("Cannot start WS2811 Repeater because PIOs do not have enough space.");
    }

    offset_repeater = pio_add_program(pio0, &ws2811Repeater_program);
    pio0->instr_mem[offset_repeater + ws2811Repeater_offset_num_bits_emulate] = pio_encode_set(pio_y, statusLEDActive ? 24 : 0);
    pio0->instr_mem[offset_repeater + ws2811Repeater_offset_wait_sideset_reset] = pio_encode_wait_gpio(1, sidesetPin);
    pio0->instr_mem[offset_repeater + ws2811Repeater_offset_wait_sideset_bit] = pio_encode_wait_gpio(1, sidesetPin);

    sm_repeater = pio_claim_unused_sm(pio0, true);
    initGPIO_repeater(dataInPin, dataOutPin);
    initSMConfig_repeater(dataInPin, sidesetPin, dataOutPin);

    // try to add second program to pio0
    if (!pio_can_add_program(pio0, &ws2812_program)) 
    {
      panic("Cannot start WS2811 sender because PIOs do not have enough space.");
    }

    sm_sender = pio_claim_unused_sm(pio0, false);
    offset_sender = pio_add_program(pio0, &ws2812_program);
    ws2812_program_init(pio0, sm_sender, offset_sender, dataOutPin, 800000, 24);

    // set the first LED in the output LED bus to light red at startup
    setStatusLEDColor(10, 0, 0);
    // as the RP2040 transfers data in a PIO state machine asynchrounusly we need to wait a little bit...
    delayMicroseconds(100);

    // change output pin to the internal RP2040 RGB status LED
    pio_sm_set_enabled(pio0, sm_sender, false);
    ws2812_program_init(pio0, sm_sender, offset_sender, STATUSLED_PIN, 800000, 24);

    // set the status LED to light red at startup
    setStatusLEDColor(10,0,0);

    runRepeaterSM();

    offset_receiver = pio_add_program(pio1, &ws2811Receiver_program);
    auto parts = getBitOffsets(this->ledsToRead);
    //static_assert(parts.first * pow(2, parts.second) == this->ledsToRead * 24);

    pio1->instr_mem[offset_receiver + ws2811Receiver_offset_num_bits_const_1] = pio_encode_set(pio_x, parts.first);
    pio1->instr_mem[offset_receiver + ws2811Receiver_offset_num_bits_const_2] = pio_encode_set(pio_y, parts.first);
    pio1->instr_mem[offset_receiver + ws2811Receiver_offset_num_bits_shift_1] = pio_encode_in(pio_null, parts.second);
    pio1->instr_mem[offset_receiver + ws2811Receiver_offset_num_bits_shift_2] = pio_encode_in(pio_null, parts.second);

    sm_receiver = pio_claim_unused_sm(pio1, true);
    dma_gather_chan = dma_claim_unused_channel(true);
    dma_ctrl_chan = dma_claim_unused_channel(true);

    init_receiverGPIO(dataInPin, sidesetPin);
    initSMConfig(dataInPin, sidesetPin, dataOutPin);
    initDMA();

    irq_set_exclusive_handler(PIO1_IRQ_0, ws2811Receiver_pio1_irq0_handler);
    pio_set_irq0_source_enabled(pio1,  pis_interrupt1, true);
    pio_set_irq0_source_enabled(pio1,  pis_interrupt2, true);
    pio_set_irq0_source_enabled(pio1,  pis_interrupt3, true);
     
    // Enable IRQ in the NVIC
    irq_set_enabled(PIO1_IRQ_0, true);
    runSM(dataInPin);
  }

  ~WS2811Processor() {
    pio_sm_set_enabled(pio1, sm_receiver, false);
    stopRepeaterSM();

    channel_config_set_chain_to(&dma_gather_conf, dma_gather_chan);
    dma_channel_abort(dma_ctrl_chan);
    dma_channel_abort(dma_gather_chan);

    dma_channel_unclaim(dma_ctrl_chan);
    dma_channel_unclaim(dma_gather_chan);

    pio_remove_program(pio1, &ws2811Receiver_program, offset_receiver);
    pio_sm_unclaim(pio1, sm_receiver);

    pio_remove_program(pio0, &ws2811Repeater_program, offset_repeater);
    pio_sm_unclaim(pio0, sm_repeater);

    // TODO: Deinit GPIO
  }

  const RGBLED getLED(uint8_t idx) const {
    if (idx>ledsToRead) ledStateToLED(0);
    return ledStateToLED(*(led_state_address+idx));
  }

  const void startGatherDma() {
    channel_config_set_chain_to(&dma_gather_conf, dma_gather_chan);
    resetRxBuffer();
  }

  inline const RGBLED ledStateToLED(const uint32_t val) const 
  {
    RGBLED result;
    switch (color_mapping) {
      case RGB:
          result.colors = {
            .r = (uint8_t)((val >> 16) & 0xFF),
            .g = (uint8_t)((val >>  8) & 0xFF),
            .b = (uint8_t)((val >>  0) & 0xFF)
          };
      default:
          result.colors = {
            .r = (uint8_t)((val >>  8) & 0xFF),
            .g = (uint8_t)((val >> 16) & 0xFF),
            .b = (uint8_t)((val >>  0) & 0xFF)
          };
    }
    return result;
  }
  
  static constexpr const uint shift(const uint val) 
  {
    switch(val)
    {
      case 2: return 1;
      case 4: return 2;
      case 8: return 3;
      case 16: return 4;
      case 32: return 5;
      case 64: return 6;
      case 128: return 7;
    }
    // invalid
    return 0;
  }


  static constexpr std::pair<uint, uint> getBitOffsets(uint numLeds) {
    const char constParts[] = { 0,12,12,18,12,15,18,21,12,27,15,17};
    const char shiftParts[] = { 1, 1, 2, 2, 3, 3, 3, 3, 4, 3, 4, 0};
    uint constPart = 0;
    uint shiftPart = 0;
    if (numLeds<=11) {
      constPart = constParts[numLeds];
      shiftPart = shiftParts[numLeds];
    }
    return std::make_pair(constPart, shiftPart);
  }

  const void getLEDs(RGBLED *leds)
  {
    for (uint i = 0; i < this->ledsToRead; i++) 
    {
      leds[i] = this->ledStateToLED(*(led_state_address+i));
    }
  }

  const uint getLedsToRead() const 
  {
    return this->ledsToRead;
  }

  const int getResetCnt() const 
  {
    return resetCnt;
  }

  bool notEnoughData() 
  {
    return ws2811Repeater_bits_to_write!=0;
  }

  void setRepeaterLEDColor(RGBLED led) 
  {
    setRepeaterLEDColor(led.colors.r, led.colors.g, led.colors.b);
  }

  // HSV->RGB conversion based on GLSL version
  // expects hsv channels defined in 0.0 .. 1.0 interval
  float fract(float x) { return x - int(x); }

  float mix(float a, float b, float t) { return a + (b - a) * t; }

  void setRepeaterLEDHSV(uint8_t hue, uint8_t sat, uint8_t bright)
  {
    float h1 = ((float)hue)/255;
    float s1 = ((float)sat)/255;
    float b1 = ((float)bright)/255;
    float r = b1 * mix(1.0, constrain(abs(fract(h1 + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s1);
    float g = b1 * mix(1.0, constrain(abs(fract(h1 + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s1);
    float b = b1 * mix(1.0, constrain(abs(fract(h1 + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s1);
    setRepeaterLEDColor(r*255, g*255, b*255);
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

  void setRepeaterLEDColor(uint8_t r, uint8_t g, uint8_t b)
  {
    uint32_t newVal;
    switch (color_mapping) {
      case RGB:
        newVal = (r << 16) + (g << 8) + b; 
      case GRB:
        newVal = (g << 16) + (r << 8) + b; 
    }
    ws2811Repeater_repeater_led_val = newVal*256;
  }

  void resetRxBuffer()
  {
    dma_channel_abort(dma_ctrl_chan);
    dma_channel_abort(dma_gather_chan);
    while (!pio_sm_is_rx_fifo_empty(pio1, sm_receiver)) pio_sm_get(pio1, sm_receiver);
    dma_channel_start(dma_ctrl_chan);
    dma_channel_start(dma_gather_chan);
  }

  void setStatusLEDColor(RGBLED led) 
  {
    setStatusLEDColor(led.colors.r, led.colors.g, led.colors.b);
  }

  void setStatusLEDColor(uint8_t r, uint8_t g, uint8_t b) 
  {
    pio_sm_put(pio0, sm_sender, (g<<24) | (r<<16) | (b<<8));
  }
  
  void reset()
  {
    //pio_sm_restart(pio1, sm_receiver);
    //startGatherDma();
    resetRxBuffer();
    //Serial.printf("restarting pio %d\r\n", sm_receiver);
    //sm_config_set_in_shift(&sm_conf, false, true, 24);  // shift left, auto push after 24 bit
  }
};


void ws2811Repeater_pio_irq0_handler() 
{
  //if (!pio_sm_is_rx_fifo_empty(pio0, 0))
  {
    ws2811Repeater_bits_to_write = pio0->rxf[0];
    if (ws2811Repeater_bits_to_write!=0)
    {
        if (ws2811Processor_instance !=NULL) ((WS2811Processor*)ws2811Processor_instance)->reset();
        WS2811Processor_ReceiveError();
    }
  }

  if (!pio_sm_is_tx_fifo_full(pio0, 0))
  {
    pio_sm_put(pio0, 0, ws2811Repeater_repeater_led_val);
  }
}

void ws2811Receiver_pio1_irq0_handler() 
{
  resetCnt = millis();

  // IRQ1 is raised when the LED receive buffer is full
  // IRQ2 is raised when a reset occurred while receiving

  if (pio_interrupt_get(pio1, 2)) 
  {
    if (pio_interrupt_get(pio1, 3))   // at least one bit received
    {
      if (ws2811Processor_instance !=NULL) ((WS2811Processor*)ws2811Processor_instance)->reset();
      WS2811Processor_ReceiveError();
      Serial.print("?");
    }
  }
  else if (pio_interrupt_get(pio1, 1)) 
  {
    // reset the buffer received flag
    if (ws2811Processor_instance !=NULL) ((WS2811Processor*)ws2811Processor_instance)->startGatherDma();
    WS2811Processor_DataReceived();
  }
  
  // reset the data received IRQ
  pio_interrupt_clear (pio1, 1 /* +0 sm_receiver */);
  // reset the data fail IRQ
  pio_interrupt_clear (pio1, 2 /* +2 sm_receiver */);
  // reset the data received IRQ
  pio_interrupt_clear (pio1, 3 /* +2 sm_receiver */);
}
