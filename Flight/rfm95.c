
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "pico/time.h"


// Define the I2C pins
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// Define the MPU6050 I2C address
#define MPU6050_ADDR 0x68

// Define the MPU6050 register addresses
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47


// Define the SPI pins
#define SPI_PORT spi0
#define SPI_SCK_PIN 18
#define SPI_MOSI_PIN 19
#define SPI_MISO_PIN 16
#define SPI_CSN_PIN 17

// Define the RFM95 register addresses
#define REG_FIFO             0x00
#define REG_OP_MODE          0x01
#define REG_FRF_MSB          0x06
#define REG_FRF_MID          0x07
#define REG_FRF_LSB          0x08
#define REG_PA_CONFIG        0x09
#define REG_LNA              0x0C
#define REG_FIFO_TX_BASE     0x0E
#define REG_FIFO_RX_BASE     0x0F
#define REG_RX_CONFIG        0x1D
#define REG_MODEM_CONFIG1    0x1E
#define REG_MODEM_CONFIG2    0x1F
#define REG_SYMB_TIMEOUT_LSB 0x20
#define REG_PREAMBLE_MSB     0x21
#define REG_PREAMBLE_LSB     0x22
#define REG_PAYLOAD_LENGTH    0x23
#define REG_MAX_PAYLOAD_LENGTH 0x24
#define REG_HOP_PERIOD        0x25
#define REG_FDEV_MSB         0x26
#define REG_FDEV_LSB         0x27
#define REG_RX_BW            0x28
#define REG_DCR_CONFIG       0x29
#define REG_MODEM_CONFIG3    0x2A
#define REG_RSSI_VALUE       0x2C
#define REG_HOP_TABLE        0x2E
#define REG_IRQ_FLAGS        0x12
#define REG_IRQ_FLAGS_MASK   0x11

// Define the RFM95 operating modes
#define MODE_SLEEP          0x00
#define MODE_STANDBY         0x01
#define MODE_FSB            0x02
#define MODE_TX             0x03
#define MODE_RX_CONTINUOUS   0x04
#define MODE_RX_SINGLE       0x05

// Define the RFM95 PA configuration
#define PA_CONFIG_BOOST_20DBM 0x80
#define PA_CONFIG_BOOST_13DBM 0x40
#define PA_CONFIG_BOOST_17DBM 0x60

// Define the RFM95 LNA configuration
#define LNA_GAIN_MAX_BOOST 0x23

// Define the RFM95 RX configuration
#define RX_CONFIG_RX_CONTINUOUS 0x01

// Define the RFM95 modem configuration
#define MODEM_CONFIG1_BW_125KHZ 0x00
#define MODEM_CONFIG1_BW_250KHZ 0x02
#define MODEM_CONFIG1_BW_500KHZ 0x04
#define MODEM_CONFIG1_BW_1MHZ 0x06
#define MODEM_CONFIG1_CR_4_5 0x00
#define MODEM_CONFIG1_CR_4_6 0x08
#define MODEM_CONFIG1_CR_4_8 0x10
#define MODEM_CONFIG1_CR_16 0x18
#define MODEM_CONFIG1_IMPLICIT_HEADER_MODE_ON 0x01
#define MODEM_CONFIG1_IMPLICIT_HEADER_MODE_OFF 0x00
#define MODEM_CONFIG2_SF_7 0x00
#define MODEM_CONFIG2_SF_8 0x01
#define MODEM_CONFIG2_SF_9 0x02
#define MODEM_CONFIG2_SF_10 0x03
#define MODEM_CONFIG2_SF_11 0x04
#define MODEM_CONFIG2_SF_12 0x05
#define MODEM_CONFIG2_SF_13 0x06
#define MODEM_CONFIG2_SF_14 0x07

// Function to write a byte to the RFM95
void rfm95_write_reg(uint8_t reg, uint8_t value) {
  uint8_t tx_buffer[2] = {reg, value};
  spi_write_blocking(SPI_PORT, tx_buffer, 2);
}

// Function to read a byte from the RFM95
uint8_t rfm95_read_reg(uint8_t reg) {
  uint8_t tx_buffer[1] = {reg | 0x80};
  uint8_t rx_buffer[1];
  spi_write_read_blocking(SPI_PORT, tx_buffer, rx_buffer, 1);
  return rx_buffer[0];
}

// Function to configure the RFM95 modem
void rfm95_config_modem(uint8_t bw, uint8_t cr, uint8_t sf, bool implicit_header) {
  // Set the bandwidth
  rfm95_write_reg(REG_MODEM_CONFIG1, bw | cr | (implicit_header ? MODEM_CONFIG1_IMPLICIT_HEADER_MODE_ON : MODEM_CONFIG1_IMPLICIT_HEADER_MODE_OFF));

  // Set the spreading factor
  rfm95_write_reg(REG_MODEM_CONFIG2, sf);
}

// Function to send a message
void rfm95_send_message(const char *message, uint8_t length) {
  // Set the payload length
  rfm95_write_reg(REG_PAYLOAD_LENGTH, length);

  // Write the message to the FIFO
  spi_write_blocking(SPI_PORT, (uint8_t *)message, length);

  // Set the RFM95 to transmit mode
  rfm95_write_reg(REG_OP_MODE, MODE_TX);

  // Wait for the transmission to complete
  while (rfm95_read_reg(REG_IRQ_FLAGS) & 0x80) {
    // Do nothing
  }

  // Clear the IRQ flags
  rfm95_write_reg(REG_IRQ_FLAGS, 0xFF);
}

// Function to receive a message
int rfm95_receive_message(char *message, uint8_t max_length) {
  // Set the RFM95 to receive mode
  rfm95_write_reg(REG_OP_MODE, MODE_RX_CONTINUOUS);

  // Wait for a message to be received
  while (!(rfm95_read_reg(REG_IRQ_FLAGS) & 0x40)) {
    // Do nothing
  }

  // Read the payload length
  uint8_t length = rfm95_read_reg(REG_PAYLOAD_LENGTH);

  // Check if the payload length is valid
  if (length > max_length) {
    length = max_length;
  }

  // Read the message from the FIFO
  spi_read_blocking(SPI_PORT, (uint8_t *)message, length);

  // Clear the IRQ flags
  rfm95_write_reg(REG_IRQ_FLAGS, 0xFF);

  return length;
}

// Function to initialize the RFM95
void rfm95_init(uint32_t frequency) {
  // Initialize the SPI port
  spi_init(SPI_PORT, 1000000);
  gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CSN_PIN, GPIO_FUNC_SIO);
  gpio_put(SPI_CSN_PIN, 1);

  // Reset the RFM95
  rfm95_write_reg(REG_OP_MODE, 0x80);
  sleep_ms(10);
  rfm95_write_reg(REG_OP_MODE, MODE_SLEEP);

  // Set the frequency
  uint32_t frf = (frequency * 1000) / 32768;
  rfm95_write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF);
  rfm95_write_reg(REG_FRF_MID, (frf >> 8) & 0xFF);
  rfm95_write_reg(REG_FRF_LSB, frf & 0xFF);

  // Set the PA configuration
  rfm95_write_reg(REG_PA_CONFIG, PA_CONFIG_BOOST_20DBM);

  // Set the LNA configuration
  rfm95_write_reg(REG_LNA, LNA_GAIN_MAX_BOOST);

  // Set the RX configuration
  rfm95_write_reg(REG_RX_CONFIG, RX_CONFIG_RX_CONTINUOUS);

  // Set the default modem configuration
  rfm95_config_modem(MODEM_CONFIG1_BW_125KHZ, MODEM_CONFIG1_CR_4_5, MODEM_CONFIG2_SF_7, false);
}

// Function to read two bytes from the MPU6050
int16_t mpu6050_read_reg(uint8_t reg) {
  uint8_t tx_buffer[1] = {reg};
  uint8_t rx_buffer[2];
  i2c_write_read_blocking(I2C_PORT, MPU6050_ADDR, tx_buffer, 1, rx_buffer, 2);
  return (rx_buffer[0] << 8) | rx_buffer[1];
}

// Function to initialize the MPU6050
void mpu6050_init() {
  // Initialize the I2C port
  i2c_init(I2C_PORT, 100000);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  // Wake up the MPU6050
  rfm95_write_reg(REG_OP_MODE, 0x80);
  sleep_ms(10);
  rfm95_write_reg(REG_OP_MODE, MODE_SLEEP);
}

// Function to read accelerometer and gyroscope data
void read_sensor_data(int16_t *accel_data, int16_t *gyro_data) {
  accel_data[0] = mpu6050_read_reg(ACCEL_XOUT_H);
  accel_data[1] = mpu6050_read_reg(ACCEL_YOUT_H);
  accel_data[2] = mpu6050_read_reg(ACCEL_ZOUT_H);
  gyro_data[0] = mpu6050_read_reg(GYRO_XOUT_H);
  gyro_data[1] = mpu6050_read_reg(GYRO_YOUT_H);
  gyro_data[2] = mpu6050_read_reg(GYRO_ZOUT_H);
}

// Function to send sensor data over LoRa
void send_sensor_data(int16_t *accel_data, int16_t *gyro_data) {
  // Create a buffer to store the sensor data
  uint8_t tx_buffer[12];

  // Pack the sensor data into the buffer
  tx_buffer[0] = (accel_data[0] >> 8) & 0xFF;
  tx_buffer[1] = accel_data[0] & 0xFF;
  tx_buffer[2] = (accel_data[1] >> 8) & 0xFF;
  tx_buffer[3] = accel_data[1] & 0xFF;
  tx_buffer[4] = (accel_data[2] >> 8) & 0xFF;
  tx_buffer[5] = accel_data[2] & 0xFF;
  tx_buffer[6] = (gyro_data[0] >> 8) & 0xFF;
  tx_buffer[7] = gyro_data[0] & 0xFF;
  tx_buffer[8] = (gyro_data[1] >> 8) & 0xFF;
  tx_buffer[9] = gyro_data[1] & 0xFF;
  tx_buffer[10] = (gyro_data[2] >> 8) & 0xFF;
  tx_buffer[11] = gyro_data[2] & 0xFF;

  // Send the data over LoRa
  rfm95_send_message((char *)tx_buffer, sizeof(tx_buffer));
}

int main() {
  stdio_init_all();

  // Initialize the SPI port, I2C port, and RFM95 module
  spi_init(SPI_PORT, 1000000);
  gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CSN_PIN, GPIO_FUNC_SIO);
  gpio_put(SPI_CSN_PIN, 1);

  i2c_init(I2C_PORT, 100000);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);

  rfm95_init(915000000); // Replace with your desired frequency

  // Configure the modem for maximum speed
  rfm95_config_modem(MODEM_CONFIG1_BW_1MHZ, MODEM_CONFIG1_CR_4_5, MODEM_CONFIG2_SF_7, true); // Implicit header enabled

  // Set the PA configuration for maximum power
  rfm95_write_reg(REG_PA_CONFIG, PA_CONFIG_BOOST_20DBM); // Adjust based on regulations

  // Initialize the MPU6050
  mpu6050_init();

  // Create arrays to store sensor data
  int16_t accel_data[3];
  int16_t gyro_data[3];

  // Create a timer for sending data
  absolute_time_t next_send_time = get_absolute_time();

  while (1) {
    // Check if it's time to send data
    if (absolute_time_diff_us(get_absolute_time(), next_send_time) >= 100000) {
      // Read sensor data
      read_sensor_data(accel_data, gyro_data);

      // Send sensor data over LoRa
      send_sensor_data(accel_data, gyro_data);

      // Update the next send time
      next_send_time = get_absolute_time() + make_timeout_time_us(100000);
    }
  }
}
