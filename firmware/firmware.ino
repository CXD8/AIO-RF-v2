#include <Arduino.h>
#include <SPI.h>

// ============================================================
// ESPC5-32E custom board pinout
// ============================================================

// Shared SPI bus
static const int PIN_SPI_SCK  = 6;
static const int PIN_SPI_MOSI = 7;
static const int PIN_SPI_MISO = 2;

// ILI9488
static const int PIN_TFT_RST = 0;
static const int PIN_TFT_CS  = 27;
static const int PIN_TFT_DC  = 26;
static const int PIN_TFT_BL  = 4;

// XPT2046
static const int PIN_TOUCH_CS  = 8;
static const int PIN_TOUCH_IRQ = 9;

// NRF1
static const int PIN_NRF1_CE  = 3;
static const int PIN_NRF1_CSN = 10;
static const int PIN_NRF1_IRQ = 5;

// NRF2
// GPIO11/12 are also UART0 TX/RX. Keep your 0R links open for UART flashing.
static const int PIN_NRF2_CE  = 11;
static const int PIN_NRF2_IRQ = 12;
static const int PIN_NRF2_CSN = 25;

// E07-400M10S / CC1101
static const int PIN_E07_CSN  = 23;
static const int PIN_E07_GDO2 = 24;
static const int PIN_E07_GDO0 = 1;

// Boot
static const int PIN_BOOT = 28;

// GPIO15 deliberately unused for H16R8/PSRAM compatibility.

// ============================================================
// SPI settings
// ============================================================

SPISettings spiTFT(20000000, MSBFIRST, SPI_MODE0);
SPISettings spiTouch(2000000, MSBFIRST, SPI_MODE0);
SPISettings spiNRF(8000000, MSBFIRST, SPI_MODE0);
SPISettings spiCC1101(8000000, MSBFIRST, SPI_MODE0);

// ============================================================
// Analyzer settings
// ============================================================

enum AnalyzerMode : uint8_t {
  MODE_CC1101_433 = 0,
  MODE_NRF_24G    = 1
};

AnalyzerMode currentMode = MODE_CC1101_433;

static const int TFT_W = 320;
static const int TFT_H = 480;

static const int PLOT_TOP = 40;
static const int PLOT_BOTTOM = 400;
static const int PLOT_H = PLOT_BOTTOM - PLOT_TOP;

// CC1101 spectrum sweep.
// The E07-400M10S matching network is intended for the 433 MHz region.
static const int CC_BINS = 160;
static const float CC_SCAN_START_MHZ = 430.0f;
static const float CC_SCAN_STOP_MHZ  = 438.0f;
int16_t ccRssi[CC_BINS];
int16_t ccPeak[CC_BINS];

// nRF24L01 channel occupancy scan.
// RF_CH 0..125 corresponds to 2400..2525 MHz.
// RPD is NOT true RSSI: it only reports energy above roughly -64 dBm.
static const int NRF_CHANNELS = 126;
static const int NRF_SAMPLES_PER_CHANNEL = 8;
uint8_t nrfOccupancy[NRF_CHANNELS];
uint8_t nrfPeak[NRF_CHANNELS];

bool backlightOn = true;

// ============================================================
// Generic GPIO/SPI helpers
// ============================================================

static inline void allCSHigh()
{
  digitalWrite(PIN_TFT_CS, HIGH);
  digitalWrite(PIN_TOUCH_CS, HIGH);
  digitalWrite(PIN_NRF1_CSN, HIGH);
  digitalWrite(PIN_NRF2_CSN, HIGH);
  digitalWrite(PIN_E07_CSN, HIGH);
}

static inline int clampInt(int v, int lo, int hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ============================================================
// ILI9488 minimal driver, RGB666 / 18-bit pixel mode
// ============================================================

void tftWriteCommand(uint8_t cmd)
{
  SPI.beginTransaction(spiTFT);
  digitalWrite(PIN_TFT_CS, LOW);
  digitalWrite(PIN_TFT_DC, LOW);
  SPI.transfer(cmd);
  digitalWrite(PIN_TFT_CS, HIGH);
  SPI.endTransaction();
}

void tftWriteCommandData(uint8_t cmd, const uint8_t *data, size_t len)
{
  SPI.beginTransaction(spiTFT);
  digitalWrite(PIN_TFT_CS, LOW);

  digitalWrite(PIN_TFT_DC, LOW);
  SPI.transfer(cmd);

  digitalWrite(PIN_TFT_DC, HIGH);
  for (size_t i = 0; i < len; i++) SPI.transfer(data[i]);

  digitalWrite(PIN_TFT_CS, HIGH);
  SPI.endTransaction();
}

void ili9488Reset()
{
  digitalWrite(PIN_TFT_RST, HIGH);
  delay(10);
  digitalWrite(PIN_TFT_RST, LOW);
  delay(20);
  digitalWrite(PIN_TFT_RST, HIGH);
  delay(150);
}

void ili9488Init()
{
  ili9488Reset();

  const uint8_t gammaPos[] = {
    0x00,0x03,0x09,0x08,0x16,0x0A,0x3F,0x78,
    0x4C,0x09,0x0A,0x08,0x16,0x1A,0x0F
  };
  tftWriteCommandData(0xE0, gammaPos, sizeof(gammaPos));

  const uint8_t gammaNeg[] = {
    0x00,0x16,0x19,0x03,0x0F,0x05,0x32,0x45,
    0x46,0x04,0x0E,0x0D,0x35,0x37,0x0F
  };
  tftWriteCommandData(0xE1, gammaNeg, sizeof(gammaNeg));

  const uint8_t pwr1[] = {0x17,0x15};
  tftWriteCommandData(0xC0, pwr1, sizeof(pwr1));

  const uint8_t pwr2[] = {0x41};
  tftWriteCommandData(0xC1, pwr2, sizeof(pwr2));

  const uint8_t vcom[] = {0x00,0x12,0x80};
  tftWriteCommandData(0xC5, vcom, sizeof(vcom));

  const uint8_t madctl[] = {0x48};
  tftWriteCommandData(0x36, madctl, sizeof(madctl));

  // ILI9488 SPI pixel interface: 18-bit / RGB666
  const uint8_t pixfmt[] = {0x66};
  tftWriteCommandData(0x3A, pixfmt, sizeof(pixfmt));

  const uint8_t ifmode[] = {0x00};
  tftWriteCommandData(0xB0, ifmode, sizeof(ifmode));

  const uint8_t frctrl[] = {0xA0};
  tftWriteCommandData(0xB1, frctrl, sizeof(frctrl));

  const uint8_t invctrl[] = {0x02};
  tftWriteCommandData(0xB4, invctrl, sizeof(invctrl));

  const uint8_t dispfunc[] = {0x02,0x02};
  tftWriteCommandData(0xB6, dispfunc, sizeof(dispfunc));

  const uint8_t imgfunc[] = {0x00};
  tftWriteCommandData(0xE9, imgfunc, sizeof(imgfunc));

  const uint8_t adj3[] = {0xA9,0x51,0x2C,0x82};
  tftWriteCommandData(0xF7, adj3, sizeof(adj3));

  tftWriteCommand(0x11);
  delay(120);
  tftWriteCommand(0x29);
  delay(20);

  digitalWrite(PIN_TFT_BL, HIGH);
}

void ili9488SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t d[4];

  d[0] = x0 >> 8; d[1] = x0;
  d[2] = x1 >> 8; d[3] = x1;
  tftWriteCommandData(0x2A, d, 4);

  d[0] = y0 >> 8; d[1] = y0;
  d[2] = y1 >> 8; d[3] = y1;
  tftWriteCommandData(0x2B, d, 4);

  tftWriteCommand(0x2C);
}

void ili9488FillRect(int x, int y, int w, int h, uint8_t r, uint8_t g, uint8_t b)
{
  if (w <= 0 || h <= 0) return;
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x + w > TFT_W) w = TFT_W - x;
  if (y + h > TFT_H) h = TFT_H - y;
  if (w <= 0 || h <= 0) return;

  ili9488SetAddrWindow(x, y, x + w - 1, y + h - 1);

  r &= 0xFC; g &= 0xFC; b &= 0xFC;

  SPI.beginTransaction(spiTFT);
  digitalWrite(PIN_TFT_CS, LOW);
  digitalWrite(PIN_TFT_DC, HIGH);

  uint32_t count = (uint32_t)w * (uint32_t)h;
  for (uint32_t i = 0; i < count; i++) {
    SPI.transfer(r);
    SPI.transfer(g);
    SPI.transfer(b);
  }

  digitalWrite(PIN_TFT_CS, HIGH);
  SPI.endTransaction();
}

void ili9488Clear(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0)
{
  ili9488FillRect(0, 0, TFT_W, TFT_H, r, g, b);
}

void drawModeHeader()
{
  if (currentMode == MODE_CC1101_433) {
    // Orange/red header = 433 MHz analyzer
    ili9488FillRect(0, 0, TFT_W, 28, 220, 60, 0);
  } else {
    // Blue header = 2.4 GHz occupancy analyzer
    ili9488FillRect(0, 0, TFT_W, 28, 0, 80, 230);
  }

  // White separator
  ili9488FillRect(0, 28, TFT_W, 2, 255, 255, 255);
}

void drawPlotGrid()
{
  ili9488FillRect(0, PLOT_TOP, TFT_W, PLOT_H, 0, 0, 0);

  // Horizontal grey grid
  for (int y = PLOT_TOP; y <= PLOT_BOTTOM; y += PLOT_H / 5) {
    ili9488FillRect(0, y, TFT_W, 1, 48, 48, 48);
  }

  // Vertical grey grid
  for (int x = 0; x < TFT_W; x += TFT_W / 8) {
    ili9488FillRect(x, PLOT_TOP, 1, PLOT_H, 35, 35, 35);
  }
}

void showModeScreen()
{
  ili9488Clear();
  drawModeHeader();
  drawPlotGrid();

  // Bottom status strip:
  // left half = 433 mode touch hint, right half = 2.4 mode hint
  ili9488FillRect(0, 430, TFT_W / 2, 50, 90, 20, 0);
  ili9488FillRect(TFT_W / 2, 430, TFT_W / 2, 50, 0, 35, 100);
}

// ============================================================
// XPT2046
// ============================================================

uint16_t xpt2046Read12(uint8_t command)
{
  SPI.beginTransaction(spiTouch);
  digitalWrite(PIN_TOUCH_CS, LOW);

  SPI.transfer(command);
  uint16_t v = (uint16_t)SPI.transfer(0x00) << 8;
  v |= SPI.transfer(0x00);

  digitalWrite(PIN_TOUCH_CS, HIGH);
  SPI.endTransaction();

  return (v >> 3) & 0x0FFF;
}

void readTouchRaw(uint16_t &x, uint16_t &y)
{
  x = xpt2046Read12(0xD0);
  y = xpt2046Read12(0x90);
}

void handleTouchModeToggle()
{
  static bool wasDown = false;
  bool down = digitalRead(PIN_TOUCH_IRQ) == LOW;

  if (down && !wasDown) {
    currentMode = (currentMode == MODE_CC1101_433) ? MODE_NRF_24G : MODE_CC1101_433;
    Serial.printf("Mode -> %s\n",
      currentMode == MODE_CC1101_433 ? "CC1101 433 MHz spectrum" : "nRF 2.4 GHz occupancy");
    showModeScreen();
  }

  wasDown = down;
}

// ============================================================
// nRF24L01 low-level driver + 2.4 GHz occupancy analyzer
// ============================================================

static const uint8_t NRF_CMD_R_REGISTER = 0x00;
static const uint8_t NRF_CMD_W_REGISTER = 0x20;
static const uint8_t NRF_CMD_NOP        = 0xFF;

static const uint8_t NRF_REG_CONFIG   = 0x00;
static const uint8_t NRF_REG_EN_AA    = 0x01;
static const uint8_t NRF_REG_RF_CH    = 0x05;
static const uint8_t NRF_REG_RF_SETUP = 0x06;
static const uint8_t NRF_REG_STATUS   = 0x07;
static const uint8_t NRF_REG_RPD      = 0x09;

uint8_t nrfReadReg(int csn, uint8_t reg)
{
  SPI.beginTransaction(spiNRF);
  digitalWrite(csn, LOW);
  SPI.transfer(NRF_CMD_R_REGISTER | (reg & 0x1F));
  uint8_t v = SPI.transfer(NRF_CMD_NOP);
  digitalWrite(csn, HIGH);
  SPI.endTransaction();
  return v;
}

void nrfWriteReg(int csn, uint8_t reg, uint8_t value)
{
  SPI.beginTransaction(spiNRF);
  digitalWrite(csn, LOW);
  SPI.transfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
  SPI.transfer(value);
  digitalWrite(csn, HIGH);
  SPI.endTransaction();
}

uint8_t nrfStatus(int csn)
{
  SPI.beginTransaction(spiNRF);
  digitalWrite(csn, LOW);
  uint8_t s = SPI.transfer(NRF_CMD_NOP);
  digitalWrite(csn, HIGH);
  SPI.endTransaction();
  return s;
}

void nrfInitScanner()
{
  digitalWrite(PIN_NRF1_CE, LOW);

  nrfWriteReg(PIN_NRF1_CSN, NRF_REG_EN_AA, 0x00);

  // 1 Mbps, 0 dBm transmit power setting.
  // TX is never enabled in analyzer mode; this just selects the RF data-rate mode.
  nrfWriteReg(PIN_NRF1_CSN, NRF_REG_RF_SETUP, 0x06);

  // PWR_UP=1, PRIM_RX=1
  nrfWriteReg(PIN_NRF1_CSN, NRF_REG_CONFIG, 0x03);

  delay(3);
}

uint8_t nrfMeasureChannelOccupancy(uint8_t channel)
{
  nrfWriteReg(PIN_NRF1_CSN, NRF_REG_RF_CH, channel);

  uint8_t hits = 0;

  for (int sample = 0; sample < NRF_SAMPLES_PER_CHANNEL; sample++) {
    // Enter RX long enough for the RPD detector to observe the channel.
    digitalWrite(PIN_NRF1_CE, HIGH);
    delayMicroseconds(180);

    if (nrfReadReg(PIN_NRF1_CSN, NRF_REG_RPD) & 0x01) hits++;

    digitalWrite(PIN_NRF1_CE, LOW);
    delayMicroseconds(40);
  }

  return (uint8_t)((hits * 100) / NRF_SAMPLES_PER_CHANNEL);
}

void scanNRF24()
{
  uint8_t bestChannel = 0;
  uint8_t bestOcc = 0;

  for (int ch = 0; ch < NRF_CHANNELS; ch++) {
    uint8_t occ = nrfMeasureChannelOccupancy((uint8_t)ch);
    nrfOccupancy[ch] = occ;
    if (occ > nrfPeak[ch]) nrfPeak[ch] = occ;

    if (occ > bestOcc) {
      bestOcc = occ;
      bestChannel = ch;
    }

    handleTouchModeToggle();
    if (currentMode != MODE_NRF_24G) return;
  }

  // Redraw plot
  ili9488FillRect(0, PLOT_TOP, TFT_W, PLOT_H, 0, 0, 0);

  const int barW = 2;
  const int graphW = NRF_CHANNELS * barW;
  const int x0 = (TFT_W - graphW) / 2;

  for (int ch = 0; ch < NRF_CHANNELS; ch++) {
    int h = (nrfOccupancy[ch] * (PLOT_H - 4)) / 100;
    int ph = (nrfPeak[ch] * (PLOT_H - 4)) / 100;
    int x = x0 + ch * barW;

    if (h > 0) {
      ili9488FillRect(x, PLOT_BOTTOM - h, barW, h, 0, 180, 255);
    }

    if (ph > 0) {
      ili9488FillRect(x, PLOT_BOTTOM - ph, barW, 2, 255, 255, 0);
    }
  }

  // Mark common Wi-Fi 2.4 GHz channel centers approximately:
  // Wi-Fi ch1=2412 -> NRF RF_CH=12, ch6=2437 -> 37, ch11=2462 -> 62.
  const uint8_t wifiCenters[] = {12, 37, 62};
  for (uint8_t i = 0; i < 3; i++) {
    int x = x0 + wifiCenters[i] * barW;
    ili9488FillRect(x, PLOT_TOP, 1, PLOT_H, 255, 80, 80);
  }

  Serial.printf("2.4G strongest: RF_CH=%u, %.0f MHz, occupancy=%u%%\n",
                bestChannel, 2400.0 + bestChannel, bestOcc);
}

// ============================================================
// CC1101 low-level driver + 433 MHz RSSI spectrum analyzer
// ============================================================

static const uint8_t CC_WRITE_BURST = 0x40;
static const uint8_t CC_READ_SINGLE = 0x80;
static const uint8_t CC_READ_BURST  = 0xC0;

// Configuration registers
static const uint8_t CC_IOCFG2   = 0x00;
static const uint8_t CC_IOCFG0   = 0x02;
static const uint8_t CC_PKTCTRL0 = 0x08;
static const uint8_t CC_FSCTRL1  = 0x0B;
static const uint8_t CC_FREQ2    = 0x0D;
static const uint8_t CC_FREQ1    = 0x0E;
static const uint8_t CC_FREQ0    = 0x0F;
static const uint8_t CC_MDMCFG4  = 0x10;
static const uint8_t CC_MDMCFG3  = 0x11;
static const uint8_t CC_MDMCFG2  = 0x12;
static const uint8_t CC_DEVIATN  = 0x15;
static const uint8_t CC_MCSM0    = 0x18;
static const uint8_t CC_FOCCFG   = 0x19;
static const uint8_t CC_AGCCTRL2 = 0x1B;
static const uint8_t CC_AGCCTRL1 = 0x1C;
static const uint8_t CC_AGCCTRL0 = 0x1D;
static const uint8_t CC_FREND1   = 0x21;
static const uint8_t CC_FREND0   = 0x22;
static const uint8_t CC_FSCAL3   = 0x23;
static const uint8_t CC_FSCAL2   = 0x24;
static const uint8_t CC_FSCAL1   = 0x25;
static const uint8_t CC_FSCAL0   = 0x26;
static const uint8_t CC_TEST2    = 0x2C;
static const uint8_t CC_TEST1    = 0x2D;
static const uint8_t CC_TEST0    = 0x2E;

// Status registers
static const uint8_t CC_PARTNUM = 0x30;
static const uint8_t CC_VERSION = 0x31;
static const uint8_t CC_RSSI    = 0x34;

// Command strobes
static const uint8_t CC_SRES  = 0x30;
static const uint8_t CC_SCAL  = 0x33;
static const uint8_t CC_SRX   = 0x34;
static const uint8_t CC_SIDLE = 0x36;
static const uint8_t CC_SNOP  = 0x3D;

bool ccWaitReady(uint32_t timeoutUs = 2500)
{
  uint32_t start = micros();
  while (digitalRead(PIN_SPI_MISO) == HIGH) {
    if ((micros() - start) > timeoutUs) return false;
  }
  return true;
}

uint8_t ccStrobe(uint8_t cmd)
{
  SPI.beginTransaction(spiCC1101);
  digitalWrite(PIN_E07_CSN, LOW);
  ccWaitReady();
  uint8_t status = SPI.transfer(cmd);
  digitalWrite(PIN_E07_CSN, HIGH);
  SPI.endTransaction();
  return status;
}

void ccWriteReg(uint8_t addr, uint8_t value)
{
  SPI.beginTransaction(spiCC1101);
  digitalWrite(PIN_E07_CSN, LOW);
  ccWaitReady();
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(PIN_E07_CSN, HIGH);
  SPI.endTransaction();
}

uint8_t ccReadReg(uint8_t addr)
{
  SPI.beginTransaction(spiCC1101);
  digitalWrite(PIN_E07_CSN, LOW);
  ccWaitReady();
  SPI.transfer(addr | CC_READ_SINGLE);
  uint8_t v = SPI.transfer(0);
  digitalWrite(PIN_E07_CSN, HIGH);
  SPI.endTransaction();
  return v;
}

uint8_t ccReadStatusReg(uint8_t addr)
{
  // CC1101 status-register reads require both READ and BURST bits.
  SPI.beginTransaction(spiCC1101);
  digitalWrite(PIN_E07_CSN, LOW);
  ccWaitReady();
  SPI.transfer(addr | CC_READ_BURST);
  uint8_t v = SPI.transfer(0);
  digitalWrite(PIN_E07_CSN, HIGH);
  SPI.endTransaction();
  return v;
}

void ccReset()
{
  digitalWrite(PIN_E07_CSN, HIGH);
  delayMicroseconds(50);
  digitalWrite(PIN_E07_CSN, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_E07_CSN, HIGH);
  delayMicroseconds(50);

  ccStrobe(CC_SRES);
  delay(2);
}

void ccInitAnalyzer()
{
  ccReset();

  // Conservative 433 MHz receiver configuration.
  // Approx. 200 kHz RX bandwidth, 2-FSK receiver path.
  ccWriteReg(CC_IOCFG2,   0x29);
  ccWriteReg(CC_IOCFG0,   0x06);
  ccWriteReg(CC_PKTCTRL0, 0x32);
  ccWriteReg(CC_FSCTRL1,  0x06);
  ccWriteReg(CC_MDMCFG4,  0xCA);
  ccWriteReg(CC_MDMCFG3,  0x83);
  ccWriteReg(CC_MDMCFG2,  0x13);
  ccWriteReg(CC_DEVIATN,  0x35);
  ccWriteReg(CC_MCSM0,    0x18);
  ccWriteReg(CC_FOCCFG,   0x16);
  ccWriteReg(CC_AGCCTRL2, 0x43);
  ccWriteReg(CC_AGCCTRL1, 0x40);
  ccWriteReg(CC_AGCCTRL0, 0x91);
  ccWriteReg(CC_FREND1,   0x56);
  ccWriteReg(CC_FREND0,   0x10);
  ccWriteReg(CC_FSCAL3,   0xE9);
  ccWriteReg(CC_FSCAL2,   0x2A);
  ccWriteReg(CC_FSCAL1,   0x00);
  ccWriteReg(CC_FSCAL0,   0x1F);
  ccWriteReg(CC_TEST2,    0x81);
  ccWriteReg(CC_TEST1,    0x35);
  ccWriteReg(CC_TEST0,    0x09);

  ccStrobe(CC_SIDLE);
}

void ccSetFrequencyMHz(float mhz)
{
  // E07/CC1101 normally uses a 26 MHz crystal.
  const float FXOSC_MHZ = 26.0f;
  uint32_t word = (uint32_t)((mhz * 65536.0f / FXOSC_MHZ) + 0.5f);

  ccWriteReg(CC_FREQ2, (word >> 16) & 0xFF);
  ccWriteReg(CC_FREQ1, (word >> 8) & 0xFF);
  ccWriteReg(CC_FREQ0, word & 0xFF);
}

int16_t ccRssiDbm()
{
  uint8_t raw = ccReadStatusReg(CC_RSSI);
  int16_t signedRaw = (raw >= 128) ? ((int16_t)raw - 256) : raw;

  // Typical CC1101 RSSI offset near 433 MHz.
  // Treat this as an estimate, not calibrated lab-grade power.
  return (signedRaw / 2) - 74;
}

int16_t ccMeasureRssi(float mhz)
{
  ccStrobe(CC_SIDLE);
  ccSetFrequencyMHz(mhz);

  // Explicit calibration for each tune step: slower but robust for a demo analyzer.
  ccStrobe(CC_SCAL);
  delayMicroseconds(800);

  ccStrobe(CC_SRX);
  delayMicroseconds(1000);

  int32_t sum = 0;
  const int samples = 3;
  for (int i = 0; i < samples; i++) {
    sum += ccRssiDbm();
    delayMicroseconds(180);
  }

  ccStrobe(CC_SIDLE);
  return (int16_t)(sum / samples);
}

void scanCC1101()
{
  int16_t bestRssi = -127;
  int bestBin = 0;

  for (int i = 0; i < CC_BINS; i++) {
    float f = CC_SCAN_START_MHZ +
      (CC_SCAN_STOP_MHZ - CC_SCAN_START_MHZ) * ((float)i / (CC_BINS - 1));

    int16_t rssi = ccMeasureRssi(f);
    ccRssi[i] = rssi;

    if (rssi > ccPeak[i]) ccPeak[i] = rssi;

    if (rssi > bestRssi) {
      bestRssi = rssi;
      bestBin = i;
    }

    handleTouchModeToggle();
    if (currentMode != MODE_CC1101_433) return;
  }

  // Redraw graph area.
  ili9488FillRect(0, PLOT_TOP, TFT_W, PLOT_H, 0, 0, 0);

  const int barW = 2;

  for (int i = 0; i < CC_BINS; i++) {
    // Graph scale: -110 dBm floor to -30 dBm top.
    int rssi = clampInt(ccRssi[i], -110, -30);
    int peak = clampInt(ccPeak[i], -110, -30);

    int h = ((rssi + 110) * (PLOT_H - 4)) / 80;
    int ph = ((peak + 110) * (PLOT_H - 4)) / 80;
    int x = i * barW;

    if (h > 0) {
      ili9488FillRect(x, PLOT_BOTTOM - h, barW, h, 0, 220, 80);
    }

    if (ph > 0) {
      ili9488FillRect(x, PLOT_BOTTOM - ph, barW, 2, 255, 80, 40);
    }
  }

  float bestFreq = CC_SCAN_START_MHZ +
    (CC_SCAN_STOP_MHZ - CC_SCAN_START_MHZ) * ((float)bestBin / (CC_BINS - 1));

  Serial.printf("433 sweep peak: %.3f MHz, approx RSSI=%d dBm\n", bestFreq, bestRssi);
}

// ============================================================
// Diagnostics / controls
// ============================================================

void resetPeakHold()
{
  for (int i = 0; i < CC_BINS; i++) ccPeak[i] = -120;
  for (int i = 0; i < NRF_CHANNELS; i++) nrfPeak[i] = 0;
  Serial.println("Peak hold reset");
}

void printDiagnostics()
{
  uint8_t ccPart = ccReadStatusReg(CC_PARTNUM);
  uint8_t ccVer  = ccReadStatusReg(CC_VERSION);

  Serial.println();
  Serial.println("=== RF board diagnostics ===");
  Serial.printf("CC1101 PARTNUM: 0x%02X\n", ccPart);
  Serial.printf("CC1101 VERSION: 0x%02X\n", ccVer);
  Serial.printf("NRF1 STATUS:    0x%02X\n", nrfStatus(PIN_NRF1_CSN));
  Serial.printf("NRF1 CONFIG:    0x%02X\n", nrfReadReg(PIN_NRF1_CSN, NRF_REG_CONFIG));
  Serial.printf("NRF2 STATUS:    0x%02X\n", nrfStatus(PIN_NRF2_CSN));
  Serial.printf("BOOT:           %d\n", digitalRead(PIN_BOOT));
  Serial.printf("E07 GDO0/GDO2:  %d/%d\n",
                digitalRead(PIN_E07_GDO0), digitalRead(PIN_E07_GDO2));
  Serial.println("============================");
}

void handleSerialControls()
{
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '1') {
      currentMode = MODE_CC1101_433;
      showModeScreen();
      Serial.println("Mode: 433 MHz CC1101 RSSI spectrum");
    }
    else if (c == '2') {
      currentMode = MODE_NRF_24G;
      showModeScreen();
      Serial.println("Mode: 2.4 GHz nRF RPD occupancy");
    }
    else if (c == 'r' || c == 'R') {
      resetPeakHold();
    }
    else if (c == 'd' || c == 'D') {
      printDiagnostics();
    }
    else if (c == 'b' || c == 'B') {
      backlightOn = !backlightOn;
      digitalWrite(PIN_TFT_BL, backlightOn ? HIGH : LOW);
    }
  }
}

// ============================================================
// Setup / loop
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_TFT_CS, OUTPUT);
  pinMode(PIN_TOUCH_CS, OUTPUT);
  pinMode(PIN_NRF1_CSN, OUTPUT);
  pinMode(PIN_NRF2_CSN, OUTPUT);
  pinMode(PIN_E07_CSN, OUTPUT);

  pinMode(PIN_TFT_RST, OUTPUT);
  pinMode(PIN_TFT_DC, OUTPUT);
  pinMode(PIN_TFT_BL, OUTPUT);

  pinMode(PIN_NRF1_CE, OUTPUT);
  pinMode(PIN_NRF2_CE, OUTPUT);

  pinMode(PIN_TOUCH_IRQ, INPUT_PULLUP);
  pinMode(PIN_NRF1_IRQ, INPUT_PULLUP);
  pinMode(PIN_NRF2_IRQ, INPUT_PULLUP);
  pinMode(PIN_E07_GDO0, INPUT);
  pinMode(PIN_E07_GDO2, INPUT);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  allCSHigh();

  digitalWrite(PIN_NRF1_CE, LOW);
  digitalWrite(PIN_NRF2_CE, LOW);
  digitalWrite(PIN_TFT_RST, HIGH);
  digitalWrite(PIN_TFT_DC, LOW);
  digitalWrite(PIN_TFT_BL, LOW);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);

  ili9488Init();
  showModeScreen();

  resetPeakHold();

  ccInitAnalyzer();
  nrfInitScanner();

  Serial.println();
  Serial.println("ESP32-C5 RF Analyzer");
  Serial.println("Touch screen: toggle analyzer mode");
  Serial.println("Serial commands:");
  Serial.println("  1 = 433 MHz CC1101 spectrum");
  Serial.println("  2 = 2.4 GHz nRF occupancy");
  Serial.println("  R = reset peak hold");
  Serial.println("  D = diagnostics");
  Serial.println("  B = toggle backlight");

  printDiagnostics();
}

void loop()
{
  handleSerialControls();
  handleTouchModeToggle();

  if (currentMode == MODE_CC1101_433) {
    scanCC1101();
  } else {
    scanNRF24();
  }
}
