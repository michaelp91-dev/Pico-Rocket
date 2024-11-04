import machine
import time
import ubinascii

# Define SPI pins
SPI_SCK_PIN = 18
SPI_MOSI_PIN = 19
SPI_MISO_PIN = 16
SPI_CSN_PIN = 17

# Define RFM95 register addresses
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_LNA = 0x0C
REG_FIFO_TX_BASE = 0x0E
REG_FIFO_RX_BASE = 0x0F
REG_RX_CONFIG = 0x1D
REG_MODEM_CONFIG1 = 0x1E
REG_MODEM_CONFIG2 = 0x1F
REG_SYMB_TIMEOUT_LSB = 0x20
REG_PREAMBLE_MSB = 0x21
REG_PREAMBLE_LSB = 0x22
REG_PAYLOAD_LENGTH = 0x23
REG_MAX_PAYLOAD_LENGTH = 0x24
REG_HOP_PERIOD = 0x25
REG_FDEV_MSB = 0x26
REG_FDEV_LSB = 0x27
REG_RX_BW = 0x28
REG_DCR_CONFIG = 0x29
REG_MODEM_CONFIG3 = 0x2A
REG_RSSI_VALUE = 0x2C
REG_HOP_TABLE = 0x2E
REG_IRQ_FLAGS = 0x12
REG_IRQ_FLAGS_MASK = 0x11

# Define RFM95 operating modes
MODE_SLEEP = 0x00
MODE_STANDBY = 0x01
MODE_FSB = 0x02
MODE_TX = 0x03
MODE_RX_CONTINUOUS = 0x04
MODE_RX_SINGLE = 0x05

# Define RFM95 PA configuration
PA_CONFIG_BOOST_20DBM = 0x80
PA_CONFIG_BOOST_13DBM = 0x40
PA_CONFIG_BOOST_17DBM = 0x60

# Define RFM95 LNA configuration
LNA_GAIN_MAX_BOOST = 0x23

# Define RFM95 RX configuration
RX_CONFIG_RX_CONTINUOUS = 0x01

# Define RFM95 modem configuration
MODEM_CONFIG1_BW_125KHZ = 0x00
MODEM_CONFIG1_BW_250KHZ = 0x02
MODEM_CONFIG1_BW_500KHZ = 0x04
MODEM_CONFIG1_BW_1MHZ = 0x06
MODEM_CONFIG1_CR_4_5 = 0x00
MODEM_CONFIG1_CR_4_6 = 0x08
MODEM_CONFIG1_CR_4_8 = 0x10
MODEM_CONFIG1_CR_16 = 0x18
MODEM_CONFIG1_IMPLICIT_HEADER_MODE_ON = 0x01
MODEM_CONFIG1_IMPLICIT_HEADER_MODE_OFF = 0x00
MODEM_CONFIG2_SF_7 = 0x00
MODEM_CONFIG2_SF_8 = 0x01
MODEM_CONFIG2_SF_9 = 0x02
MODEM_CONFIG2_SF_10 = 0x03
MODEM_CONFIG2_SF_11 = 0x04
MODEM_CONFIG2_SF_12 = 0x05
MODEM_CONFIG2_SF_13 = 0x06
MODEM_CONFIG2_SF_14 = 0x07

# Initialize SPI
spi = machine.SPI(0, baudrate=1000000, polarity=0, phase=0)
cs = machine.Pin(SPI_CSN_PIN, machine.Pin.OUT)
cs.value(1)

# Function to write a byte to the RFM95
def rfm95_write_reg(reg, value):
    cs.value(0)
    spi.write(bytearray([reg, value]))
    cs.value(1)

# Function to read a byte from the RFM95
def rfm95_read_reg(reg):
    cs.value(0)
    spi.write(bytearray([reg | 0x80]))
    data = spi.read(1)
    cs.value(1)
    return data[0]

# Function to configure the RFM95 modem
def rfm95_config_modem(bw, cr, sf, implicit_header):
    # Set the bandwidth
    rfm95_write_reg(REG_MODEM_CONFIG1, bw | cr | (implicit_header * MODEM_CONFIG1_IMPLICIT_HEADER_MODE_ON))

    # Set the spreading factor
    rfm95_write_reg(REG_MODEM_CONFIG2, sf)

# Function to send a message
def rfm95_send_message(message, length):
    # Set the payload length
    rfm95_write_reg(REG_PAYLOAD_LENGTH, length)

    # Write the message to the FIFO
    spi.write(bytearray(message))

    # Set the RFM95 to transmit mode
    rfm95_write_reg(REG_OP_MODE, MODE_TX)

    # Wait for the transmission to complete
    while (rfm95_read_reg(REG_IRQ_FLAGS) & 0x80):
        pass

    # Clear the IRQ flags
    rfm95_write_reg(REG_IRQ_FLAGS, 0xFF)

# Function to receive a message
def rfm95_receive_message(max_length):
    # Set the RFM95 to receive mode
    rfm95_write_reg(REG_OP_MODE, MODE_RX_CONTINUOUS)

    # Wait for a message to be received
    while not (rfm95_read_reg(REG_IRQ_FLAGS) & 0x40):
        pass

    # Read the payload length
    length = rfm95_read_reg(REG_PAYLOAD_LENGTH)

    # Check if the payload length is valid
    if length > max_length:
        length = max_length

    # Read the message from the FIFO
    data = spi.read(length)

    # Clear the IRQ flags
    rfm95_write_reg(REG_IRQ_FLAGS, 0xFF)

    return data

def rfm95_init(frequency):
    # Reset the RFM95
    rfm95_write_reg(REG_OP_MODE, 0x80)
    time.sleep(0.01)
    rfm95_write_reg(REG_OP_MODE, MODE_SLEEP)

    # Set the frequency
    frf = (frequency * 1000) / 32768
    rfm95_write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF)
    rfm95_write_reg(REG_FRF_MID, (frf >> 8) & 0xFF)
    rfm95_write_reg(REG_FRF_LSB, frf & 0xFF)

    # Set the PA configuration
    rfm95_write_reg(REG_PA_CONFIG, PA_CONFIG_BOOST_20DBM)

    # Set the LNA configuration
    rfm95_write_reg(REG_LNA, LNA_GAIN_MAX_BOOST)

    # Set the RX configuration
    rfm95_write_reg(REG_RX_CONFIG, RX_CONFIG_RX_CONTINUOUS)

    # Set the default modem configuration
    rfm95_config_modem(MODEM_CONFIG1_BW_125KHZ, MODEM_CONFIG1_CR_4_5, MODEM_CONFIG2_SF_7, False)

# Main program
def main():
    # Initialize the RFM95
    rfm95_init(915000000)  # Replace with your desired frequency

    # Configure the modem for maximum speed
    rfm95_config_modem(MODEM_CONFIG1_BW_1MHZ, MODEM_CONFIG1_CR_4_5, MODEM_CONFIG2_SF_7, True)  # Implicit header enabled

    # Set the PA configuration for maximum power
    rfm95_write_reg(REG_PA_CONFIG, PA_CONFIG_BOOST_20DBM)  # Adjust based on regulations

    # Create a timer for receiving messages
    next_receive_time = time.ticks_ms()

    while True:
        # Check if it's time to receive the next message
        if time.ticks_diff(time.ticks_ms(), next_receive_time) >= 100:  # 100 milliseconds = 0.1 seconds
            # Receive a message
            data = rfm95_receive_message(256)
            if data:
                print("Received message:", data.decode())

            # Update the next receive time
            next_receive_time = time.ticks_ms() + 100  # Add 0.1 seconds to the current time

if __name__ == "__main__":
    main()
