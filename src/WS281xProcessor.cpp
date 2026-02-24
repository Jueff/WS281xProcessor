#include <stdio.h>
#include <array>
#include <cmath>
#include <functional>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws281xProcessor.h"
#include "WS281xBase.h"
#include "ws281xReceiver.pio.h"
#include "ws281xRepeater.pio.h"
#include "WS2812Sender.h"
#include "rp2040_pio.h"
#include <arduino.h>

// Globale Variablen
void* ws281xProcessor_instance = NULL;
uint32_t resetCnt = 0;
static uint32_t ws281xRepeater_repeater_led_val = 0;
static uint32_t ws281xRepeater_bits_to_write = 24;

// IRQ Handler Implementierungen
void ws281xRepeater_pio_irq0_handler()
{
  ws281xRepeater_bits_to_write = pio0->rxf[0];
  if (ws281xRepeater_bits_to_write != 0)
  {
    if (ws281xProcessor_instance != NULL) ((WS281xProcessor*)ws281xProcessor_instance)->reset();
    WS281xProcessor_ReceiveError();
  }

  if (!pio_sm_is_tx_fifo_full(pio0, 0))
  {
    pio_sm_put(pio0, 0, ws281xRepeater_repeater_led_val);
  }
}

void ws281xReceiver_pio1_irq0_handler()
{
  resetCnt = millis();

  if (pio_interrupt_get(pio1, 2))
  {
    if (pio_interrupt_get(pio1, 3))
    {
      auto proc = (WS281xProcessor*)ws281xProcessor_instance;
      if (proc != NULL)
      {
        proc->reset();
        proc->notifyReceiveError();
      }
      WS281xProcessor_ReceiveError();
    }
  }
  else if (pio_interrupt_get(pio1, 1))
  {
    auto proc = (WS281xProcessor*)ws281xProcessor_instance;
    if (proc != NULL)
    {
      proc->startGatherDma();
      proc->notifyDataReceived();
    }
    WS281xProcessor_DataReceived();
  }

  pio_interrupt_clear(pio1, 1);
  pio_interrupt_clear(pio1, 2);
  pio_interrupt_clear(pio1, 3);
}

// Methodenimplementierungen
WS281xProcessor::WS281xProcessor()
  : led_state_address(nullptr), ledsToSkip(0), ledsToRead(0), statusLEDActive(false)
{
}

WS281xProcessor::~WS281xProcessor() {
  pio_sm_set_enabled(pio1, sm_receiver, false);
  stopRepeaterSM();

  channel_config_set_chain_to(&dma_gather_conf, dma_gather_chan);
  dma_channel_abort(dma_ctrl_chan);
  dma_channel_abort(dma_gather_chan);

  dma_channel_unclaim(dma_ctrl_chan);
  dma_channel_unclaim(dma_gather_chan);

  pio_remove_program(pio1, &ws281xReceiver_program, offset_receiver);
  pio_sm_unclaim(pio1, sm_receiver);

  pio_remove_program(pio0, &ws281xRepeater_program, offset_repeater);
  pio_sm_unclaim(pio0, sm_repeater);

  if (led_state_address) free((void*)led_state_address);
  // TODO: Deinit GPIO
}

void WS281xProcessor::init(uint8_t dataInPin, uint8_t sidesetPin, uint8_t dataOutPin, uint8_t ledsToRead, uint8_t ledsToSkip, WS281xColorMapping mapping, bool statusLEDActive)
{
  if (this->ledsToSkip != 0 || this->ledsToRead != 0)
  {
    panic("Cannot init WS281x processor twice!");
    return;
  }
  this->ledsToSkip = ledsToSkip;
  this->ledsToRead = ledsToRead;
  this->statusLEDActive = statusLEDActive;
  this->led_state_address = (uint32_t*)malloc(ledsToRead * sizeof(uint32_t));
  this->color_mapping = mapping;
  ws281xProcessor_instance = this;

  if (!pio_can_add_program(pio1, &ws281xReceiver_program)) {
    panic("Cannot start WS281x client because PIOs do not have enough space.");
  }

  if (!pio_can_add_program(pio0, &ws281xRepeater_program))
  {
    panic("Cannot start WS281x Repeater because PIOs do not have enough space.");
  }

  offset_repeater = pio_add_program(pio0, &ws281xRepeater_program);
  pio0->instr_mem[offset_repeater + ws281xRepeater_offset_num_bits_emulate] = pio_encode_set(pio_y, statusLEDActive ? 24 : 0);
  pio0->instr_mem[offset_repeater + ws281xRepeater_offset_wait_sideset_reset] = pio_encode_wait_gpio(1, sidesetPin);
  pio0->instr_mem[offset_repeater + ws281xRepeater_offset_wait_sideset_bit] = pio_encode_wait_gpio(1, sidesetPin);

  sm_repeater = pio_claim_unused_sm(pio0, true);
  initGPIO_repeater(dataInPin, dataOutPin);
  initSMConfig_repeater(dataInPin, sidesetPin, dataOutPin);

  if (!pio_can_add_program(pio0, &ws2812sender_program))
  {
    panic("Cannot start WS2812 sender because PIOs do not have enough space.");
  }
  auto sender = new WS2812Sender();
  if (sender->init(dataOutPin)) {
    sender->setStatusLEDColor(10, 0, 0);
    delayMicroseconds(100);
  }
  delete sender;

  runRepeaterSM();

  offset_receiver = pio_add_program(pio1, &ws281xReceiver_program);
  auto parts = getBitOffsets(this->ledsToRead);

  pio1->instr_mem[offset_receiver + ws281xReceiver_offset_num_bits_const_1] = pio_encode_set(pio_x, parts.first);
  pio1->instr_mem[offset_receiver + ws281xReceiver_offset_num_bits_const_2] = pio_encode_set(pio_y, parts.first);
  pio1->instr_mem[offset_receiver + ws281xReceiver_offset_num_bits_shift_1] = pio_encode_in(pio_null, parts.second);
  pio1->instr_mem[offset_receiver + ws281xReceiver_offset_num_bits_shift_2] = pio_encode_in(pio_null, parts.second);

  sm_receiver = pio_claim_unused_sm(pio1, true);
  dma_gather_chan = dma_claim_unused_channel(true);
  dma_ctrl_chan = dma_claim_unused_channel(true);

  init_receiverGPIO(dataInPin, sidesetPin);
  initSMConfig(dataInPin, sidesetPin, dataOutPin);
  initDMA();

  irq_set_exclusive_handler(PIO1_IRQ_0, ws281xReceiver_pio1_irq0_handler);
  pio_set_irq0_source_enabled(pio1, pis_interrupt1, true);
  pio_set_irq0_source_enabled(pio1, pis_interrupt2, true);
  pio_set_irq0_source_enabled(pio1, pis_interrupt3, true);

  irq_set_enabled(PIO1_IRQ_0, true);
  runSM(dataInPin);
}

const WS281xBase::RGBLED WS281xProcessor::getLED(uint8_t idx) const {
  if (idx > ledsToRead) return ledStateToLED(0);
  return ledStateToLED(*(led_state_address + idx));
}

const void WS281xProcessor::startGatherDma() {
  channel_config_set_chain_to(&dma_gather_conf, dma_gather_chan);
  resetRxBuffer();
}

inline const WS281xBase::RGBLED WS281xProcessor::ledStateToLED(const uint32_t val) const
{
  RGBLED result;
  switch (color_mapping) {
  case RGB:
    result.colors = {
        .r = (uint8_t)((val >> 16) & 0xFF),
        .g = (uint8_t)((val >> 8) & 0xFF),
        .b = (uint8_t)((val >> 0) & 0xFF)
    };
    break;
  case GRB:
  default:
    result.colors = {
        .r = (uint8_t)((val >> 8) & 0xFF),
        .g = (uint8_t)((val >> 16) & 0xFF),
        .b = (uint8_t)((val >> 0) & 0xFF)
    };
    break;
  }
  return result;
}

void WS281xProcessor::registerDataReceivedCallback(std::function<void()> cb)
{
  dataReceivedCallback = cb;
}

void WS281xProcessor::registerReceiveErrorCallback(std::function<void()> cb)
{
  receiveErrorCallback = cb;
}

void WS281xProcessor::notifyDataReceived()
{
  if (dataReceivedCallback) dataReceivedCallback();
}

void WS281xProcessor::notifyReceiveError()
{
  if (receiveErrorCallback) receiveErrorCallback();
}

constexpr const uint WS281xProcessor::shift(const uint val)
{
  switch (val)
  {
  case 2: return 1;
  case 4: return 2;
  case 8: return 3;
  case 16: return 4;
  case 32: return 5;
  case 64: return 6;
  case 128: return 7;
  }
  return 0;
}

constexpr std::pair<uint, uint> WS281xProcessor::getBitOffsets(uint numLeds) {
  const char constParts[] = { 0,12,12,18,12,15,18,21,12,27,15,17 };
  const char shiftParts[] = { 1, 1, 2, 2, 3, 3, 3, 3, 4, 3, 4, 0 };
  uint constPart = 0;
  uint shiftPart = 0;
  if (numLeds <= 11) {
    constPart = constParts[numLeds];
    shiftPart = shiftParts[numLeds];
  }
  return std::make_pair(constPart, shiftPart);
}

const void WS281xProcessor::getLEDs(RGBLED* leds)
{
  for (uint i = 0; i < this->ledsToRead; i++)
  {
    leds[i] = this->ledStateToLED(*(led_state_address + i));
  }
}

const uint WS281xProcessor::getLedsToRead() const
{
  return this->ledsToRead;
}

const int WS281xProcessor::getResetCnt() const
{
  return resetCnt;
}

bool WS281xProcessor::notEnoughData()
{
  return ws281xRepeater_bits_to_write != 0;
}

void WS281xProcessor::setRepeaterLEDColor(RGBLED led)
{
  setRepeaterLEDColor(led.colors.r, led.colors.g, led.colors.b);
}

void WS281xProcessor::setRepeaterLEDHSV(uint8_t hue, uint8_t sat, uint8_t bright)
{
  setRepeaterLEDColor(HsvToRgb(hue, sat, bright));
}

void WS281xProcessor::setRepeaterLEDColor(uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t newVal;
  switch (color_mapping) {
  case RGB:
    newVal = (r << 16) + (g << 8) + b;
    break;
  case GRB:
    newVal = (g << 16) + (r << 8) + b;
    break;
  }
  ws281xRepeater_repeater_led_val = newVal * 256;
}

void WS281xProcessor::resetRxBuffer()
{
  dma_channel_abort(dma_ctrl_chan);
  dma_channel_abort(dma_gather_chan);
  while (!pio_sm_is_rx_fifo_empty(pio1, sm_receiver)) pio_sm_get(pio1, sm_receiver);
  dma_channel_start(dma_ctrl_chan);
  dma_channel_start(dma_gather_chan);
}

void WS281xProcessor::reset()
{
  resetRxBuffer();
}

// Inline Methoden (aus dem Header, jetzt als normale Methoden)
void WS281xProcessor::init_receiverGPIO(uint8_t dataInPin, uint8_t sidesetPin) {
  pio_sm_set_consecutive_pindirs(pio1, sm_receiver, dataInPin, 1, GPIO_IN);
  pinMode(dataInPin, INPUT_PULLDOWN);
  pio_gpio_init(pio1, dataInPin);
  pio_sm_set_consecutive_pindirs(pio1, sm_receiver, sidesetPin, 1, GPIO_OUT);
  pio_gpio_init(pio1, sidesetPin);
}

void WS281xProcessor::initSMConfig(uint8_t dataInPin, uint8_t sidesetPin, uint8_t dataOutPin) {
  sm_conf = ws281xReceiver_program_get_default_config(offset_receiver);

  sm_config_set_in_pins(&sm_conf, dataInPin);
  sm_config_set_jmp_pin(&sm_conf, dataInPin);
  sm_config_set_sideset_pins(&sm_conf, sidesetPin);

  sm_config_set_in_shift(&sm_conf, false, true, 24);
  sm_config_set_out_shift(&sm_conf, false, false, 0);
  sm_config_set_fifo_join(&sm_conf, PIO_FIFO_JOIN_RX);

  sm_config_set_clkdiv(&sm_conf, 1);
}

void WS281xProcessor::initDMA() {
  dma_ctrl_conf = dma_channel_get_default_config(dma_ctrl_chan);
  dma_gather_conf = dma_channel_get_default_config(dma_gather_chan);

  channel_config_set_transfer_data_size(&dma_ctrl_conf, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_ctrl_conf, false);
  channel_config_set_write_increment(&dma_ctrl_conf, false);
  dma_channel_configure(
    dma_ctrl_chan,
    &dma_ctrl_conf,
    &dma_hw->ch[dma_gather_chan].al2_write_addr_trig,
    &led_state_address,
    1,
    false
  );

  channel_config_set_transfer_data_size(&dma_gather_conf, DMA_SIZE_32);
  channel_config_set_read_increment(&dma_gather_conf, false);
  channel_config_set_write_increment(&dma_gather_conf, true);
  channel_config_set_dreq(&dma_gather_conf, pio_get_dreq(pio1, sm_receiver, false));
  channel_config_set_chain_to(&dma_gather_conf, dma_ctrl_chan);
  dma_channel_configure(
    dma_gather_chan,
    &dma_gather_conf,
    led_state_address,
    &pio1->rxf[sm_receiver],
    this->ledsToRead,
    false
  );

  dma_channel_start(dma_ctrl_chan);
}

void WS281xProcessor::runSM(uint8_t dataInPin)
{
  pio_sm_clear_fifos(pio1, sm_receiver);
  pio_sm_init(pio1, sm_receiver, offset_receiver, &sm_conf);

  pio_sm_exec_wait_blocking(pio1, sm_receiver, pio_encode_set(pio_y, 20));
  pio_sm_exec_wait_blocking(pio1, sm_receiver, pio_encode_mov(pio_osr, pio_y));
  pio_sm_exec_wait_blocking(pio1, sm_receiver, pio_encode_out(pio_null, 5));

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

void WS281xProcessor::initGPIO_repeater(uint8_t dataInPin, uint8_t dataOutPin)
{
  pio_sm_set_consecutive_pindirs(pio0, sm_repeater, dataInPin, 1, GPIO_IN);
  pio_gpio_init(pio0, dataInPin);

  pio_sm_set_consecutive_pindirs(pio0, sm_repeater, dataOutPin, 1, GPIO_OUT);
  pio_gpio_init(pio0, dataOutPin);
}

void WS281xProcessor::initSMConfig_repeater(uint8_t dataInPin, uint8_t sidesetPin, uint8_t dataOutPin)
{
  sm_conf_repeater = ws281xRepeater_program_get_default_config(offset_repeater);

  sm_config_set_in_pins(&sm_conf_repeater, dataInPin);
  sm_config_set_jmp_pin(&sm_conf_repeater, dataInPin);
  sm_config_set_out_pins(&sm_conf_repeater, dataOutPin, 1);
  sm_config_set_set_pins(&sm_conf_repeater, dataOutPin, 1);
  sm_config_set_sideset_pins(&sm_conf_repeater, sidesetPin);

  sm_config_set_in_shift(&sm_conf_repeater, false, false, 0);
  sm_config_set_out_shift(&sm_conf_repeater, false, true, 32);
  sm_config_set_fifo_join(&sm_conf_repeater, PIO_FIFO_JOIN_NONE);

  sm_config_set_clkdiv(&sm_conf_repeater, 2);

  irq_set_exclusive_handler(PIO0_IRQ_0, ws281xRepeater_pio_irq0_handler);

  pio_set_irq0_source_enabled(pio0, pis_sm0_tx_fifo_not_full, true);
  pio_set_irq0_source_enabled(pio0, pis_sm0_rx_fifo_not_empty, true);
}

void WS281xProcessor::runRepeaterSM() {
  pio_sm_clear_fifos(pio0, sm_repeater);
  pio_sm_init(pio0, sm_repeater, offset_repeater, &sm_conf_repeater);

  pio_sm_put(pio0, 0, ledsToSkip * 24);

  pio_sm_exec_wait_blocking(pio0, sm_repeater, pio_encode_pull(false, false));
  pio_sm_exec_wait_blocking(pio0, sm_repeater, pio_encode_mov(pio_x, pio_osr));
  pio_sm_exec_wait_blocking(pio0, sm_repeater, pio_encode_mov(pio_isr, pio_x));

  irq_set_enabled(PIO0_IRQ_0, true);

  pio_sm_set_enabled(pio0, sm_repeater, true);
}

void WS281xProcessor::stopRepeaterSM() {
  irq_set_enabled(PIO0_IRQ_0, false);
  pio_sm_set_enabled(pio0, sm_repeater, false);
}