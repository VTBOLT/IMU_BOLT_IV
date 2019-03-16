// Enable Non-Volatile Memory Storage
// If defined, the FlashStorage library must be installed
#define ENABLE_NVRAM_STORAGE true

// Serial Port Config
#define SERIAL_BAUD_RATE 115200
#define DEBUG SERIAL_PORT_USBVIRTUAL // Equivalent to SerialUSB

// User LED Config
#define HW_LED_PIN 13
#define UART_BLINK_RATE 1000

// IMU Config Settings
#define DMP_SAMPLE_RATE           100 // DMP sample rate (4-200 Hz)
#define IMU_COMPASS_SAMPLE_RATE   100 // Magnetometer sample rate (4-100 Hz)
#define IMU_AG_SAMPLE_RATE        100 // Accel/Gyro sample rate (4Hz-1kHz)
#define IMU_GYRO_FSR              250 // Gyro full-scale range +/-(250, 500, 1000, or 2000)
#define IMU_ACCEL_FSR             2   // Accel full-scale ranve +/-(2, 4, 8, or 16)
#define IMU_AG_LPF                5   // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)

// Hardware Definitions
// NEVER CHANGE
#define SD_CHIP_SELECT_PIN 38

// Software Definitions
#define DEBUG_ENABLE true // Set to false to disable debug statements through USB and it reduce program size
#define DEBUG_OUTPUT_RATE 100 // Output data will be updated 100ms
