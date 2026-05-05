#define USER_SETUP_INFO "ESP32-S3 ST7789V SO101"

// Driver chip
#define ST7789_DRIVER

// Resolution
#define TFT_WIDTH  240
#define TFT_HEIGHT 240

// SPI pins (J1 left side)
#define TFT_MOSI 11   // SDA (data)
#define TFT_SCLK 12   // SCL (clock)
#define TFT_CS   10   // CS1 (display chip select)
#define TFT_DC    6   // DC (data/command select)
#define TFT_RST   7   // RES (reset)
#define TFT_MISO 13   // FSO (font chip readback)

// Backlight
#define TFT_BL    5

// Color byte order (critical!)
#define TFT_RGB_ORDER TFT_RGB

// LCD polarity (IPS screen usually needs)
#define TFT_INVERSION_ON

// SPI frequency
#define SPI_FREQUENCY  40000000   // 40MHz refresh
#define SPI_READ_FREQ   8000000   // 8MHz read font

// Load fonts
#define LOAD_GLCD
#define LOAD_FONT2

// Support SPI transactions (required for multi-device shared bus)
#define SUPPORT_TRANSACTIONS
