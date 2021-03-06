// Enable Non-Volatile Memory Storage
// If defined, the FlashStorage library must be installed
#define ENABLE_NVRAM_STORAGE true

// Serial Port Config
#define SERIAL_BAUD_RATE 57600
#define DEBUG_BAUD_RATE 57600
#define DEBUG SERIAL_PORT_USBVIRTUAL // Use USB Serial Port
#define HW_SERIAL SERIAL_PORT_HARDWARE // Use TX RX pins on PCB

// User LED Config
#define HW_LED_PIN 13
#define UART_BLINK_RATE 1000

// IMU Config Settings
#define DMP_SAMPLE_RATE           100 // DMP sample rate (4-200 Hz)
#define IMU_COMPASS_SAMPLE_RATE   100 // Magnetometer sample rate (4-100 Hz)
#define IMU_AG_SAMPLE_RATE        100 // Accel/Gyro sample rate (4Hz-1kHz)
#define IMU_GYRO_FSR              250 // Gyro full-scale range +/-(250, 500, 1000, or 2000)
#define IMU_ACCEL_FSR             2   // Accel full-scale range +/-(2, 4, 8, or 16)
#define IMU_AG_LPF                5   // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)

// Hardware Definitions
// NEVER CHANGE
#define SD_CHIP_SELECT_PIN 38

// Software Definitions
#define DEBUG_ENABLE true // Set to false to disable debug statements through USB and it reduce program size
#define DEBUG_OUTPUT_RATE 100 // Output data will be updated 100ms
#define TRANSMIT_OUTPUT_INCREMENT 10 // Data will be output every 100ms

// SD Logging Config
#define LOG_FILE_INDEX_MAX 999 // Max number of "logXXX.txt" files
#define LOG_FILE_PREFIX "log"  // Prefix name for log files
#define LOG_FILE_SUFFIX "csv"  // Suffix name for log files
#define SD_MAX_FILE_SIZE 5000000 // 5MB max file size, increment to next file before surpassing
#define SD_LOG_WRITE_BUFFER_SIZE 1024 // Experimentally tested to produce 100Hz logs
