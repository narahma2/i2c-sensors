from labjack import ljm
import numpy as np
import socket
import time


def readSHT31(hostname):
    """
    Check datasheet for SHT3x-DIS for further detail; check Adafruit
    website to find the datasheet.
    Current script is setup for single-shot mode, however periodic is also
    possible, see Section 4.5 of datasheet for commands.
    """
    # Retrieve LabJack IP address using hostname
    lbjt7_ip = socket.gethostbyname(hostname)

    # Initiation connection to the LabJack
    handle = ljm.openS('ANY', 'TCP', lbjt7_ip)

    # On/off voltages and PIN used for power
    power = 'DAC0'
    on = 3.3
    off = 0

    # Turn on sensor
    ljm.eWriteName(handle, power, on)

    # Wait for power-up time (t_PU, Table 4)
    time.sleep(1.1E-3)

    # SDA: FIO1, SCL: FIO0
    ljm.eWriteName(handle, 'I2C_SDA_DIONUM', 1)
    ljm.eWriteName(handle, 'I2C_SCL_DIONUM', 0)

    # Speed throttle is inversely proportional to clock frequency. 0 = max.
    ljm.eWriteName(handle, "I2C_SPEED_THROTTLE", 65000)  # Speed throttle = 65516 (~100 kHz)

    # Options bits:
    # bit0: Reset the I2C bus.
    # bit1: Restart w/o stop
    # bit2: Disable clock stretching.
    ljm.eWriteName(handle, 'I2C_OPTIONS', 0)  # Options = 0

    # The SHT3x address could be 0x44 or 0x45 depending on the address pin voltage
    # A slave address of 0x44 indicates the ADDR pin is connected to a logic low
    # See Table 7
    ljm.eWriteName(handle, "I2C_SLAVE_ADDRESS", 0x44)

    # Start with a single shot write command to the SHT3x sensor.
    numTX = 2
    numRX = 0
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', numTX)  # Set the number of bytes to transmit
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_RX', numRX)  # Set the number of bytes to receive

    # Set the TX bytes
    # Single shot mode: medium repeatability, clock stretching disabled
    # See Table 8
    aBytes = [0x24, 0x0B]
    ljm.eWriteNameByteArray(handle, 'I2C_DATA_TX', numTX, aBytes)
    ljm.eWriteName(handle, 'I2C_GO', 1)  # Do the I2C communications

    # The sensor needs at least 15ms for the measurement; wait 20ms
    time.sleep(20E-3)

    # Read-only to obtain measurements
    numTX = 0
    numRX = 6
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', numTX)  # Set the number of bytes to transmit
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_RX', numRX)  # Set the number of bytes to receive
    ljm.eWriteName(handle, 'I2C_GO', 1)  # Do the I2C communications

    # Read the RX bytes
    # aBytes[0,1]: temperature, aBytes[2]: CRC checksum
    # aBytes[3,4]: humidity, aBytes[5]: CRC checksum
    # NOTE: the sensor is big-endian (most significant bit aka MSB is first)
    aBytes = ljm.eReadNameByteArray(handle, 'I2C_DATA_RX', numRX)

    # Combine the temperature and humidity packets
    T_bin16 = f'{aBytes[0]:08b}{aBytes[1]:08b}'
    RH_bin16 = f'{aBytes[3]:08b}{aBytes[4]:08b}'

    # Convert to unsigned 16-bit integers
    # https://stackoverflow.com/a/8928256
    T_uint16 = np.uint16(int(T_bin16, 2))
    RH_uint16 = np.uint16(int(RH_bin16, 2))

    # Convert raw data to physical units
    # Linearization/compensation already done on-chip
    # See Section 4.13 (Conversion of Signal Output)
    T_degC = -45 + 175*(T_uint16 / (2**16 - 1))
    RH_perc = 100*(RH_uint16 / (2**16 - 1))

    # Turn off sensor
    ljm.eWriteName(handle, power, off)

    # Close handle
    ljm.close(handle)

    # Round off values
    T_degC = np.round(T_degC, 2)
    RH_perc = np.round(RH_perc, 2)

    # Return as (temperature [degC], relative humidity [%])
    return (T_degC, RH_perc)


def readMS8607(hostname):
    # Retrieve LabJack IP address using hostname
    #lbjt7_ip = socket.gethostbyname(hostname)

    # Initiate connection to the LabJack device
    handle = ljm.openS('ANY', 'USB', 'ANY')

    # On/off voltages and PIN used for power
    power = 'DAC0'
    on = 5
    off = 0

    # Turn on sensor
    ljm.eWriteName(handle, power, on)

    # SDA: FIO1, SCL: FIO0
    ljm.eWriteName(handle, 'I2C_SDA_DIONUM', 1)
    ljm.eWriteName(handle, 'I2C_SCL_DIONUM', 0)

    # Speed throttle is inversely proportional to clock frequency
    # 65516 ~ 100 kHz
    ljm.eWriteName(handle, 'I2C_SPEED_THROTTLE', 65516)

    # Options bits:
    #   - bit0: Reset the I2C bus.
    #   - bit1: Restart w/o stop
    #   - bit2: Disable clock stretching.
    ljm.eWriteName(handle, "I2C_OPTIONS", 0b010)

    # Pressure/temperature address: 0x76 (1110110)
    ljm.eWriteName(handle, 'I2C_SLAVE_ADDRESS', 0x76)

    # Let's start with a reset command
    # Set number of bytes to transmit/receive
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', 1)
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_RX', 0)

    # Set the TX bytes
    numBytes = 1

    # Reset: 0x1E (00011110)
    aBytes = [0x1E]
    ljm.eWriteNameByteArray(handle, 'I2C_DATA_TX', numBytes, aBytes)
    ljm.eWriteName(handle, 'I2C_GO', 1) # Do the I2C communications

    # Initialize pressure/temperature calibration values C1--C6
    pt_cal = [0] * 6

    # Addresses for C1--C6
    pt_cal_adrs = [0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC]

    for i, adr in enumerate(pt_cal_adrs):
        # Read in the factory-set calibration values
        numTX = 1
        numRX = 2
        ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', numTX)
        ljm.eWriteName(handle, 'I2C_NUM_BYTES_RX', numRX)

        # Set the TX bytes. We are sending 1 byte for the address.
        aBytes = [adr]
        ljm.eWriteNameByteArray(handle, 'I2C_DATA_TX', numTX, aBytes)
        ljm.eWriteName(handle, 'I2C_GO', 1)

        # Read the RX bytes.
        # aBytes[0] and aBytes[1] will contain the data
        # NOTE: the sensor is big-endian (most significant bit aka MSB is first)
        aBytes = ljm.eReadNameByteArray(handle, 'I2C_DATA_RX', numRX)

        # Combine the 2 packets of 8-bits into a 16-bit binary number
        bin16 = f'{aBytes[0]:08b}{aBytes[1]:08b}'

        # Convert the 16-bit binary string into an (unsigned!) 16-bit integer
        # Store as an unsigned 16-bit int as suggested
        # https://stackoverflow.com/a/8928256
        pt_cal[i] = int(bin16, 2)

    # Uncompensated pressure/temperature [D1, D2]
    pt_uncomp = [0] * 2

    # Pressure/temperature addresses (OSR = 1024)
    pt_adrs = [0x44, 0x54]

    # Initiate measurement
    for i, adr in enumerate(pt_adrs):
        numTX = 1
        ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', numTX)

        # Set the TX bytes. We are sending 1 byte for the address.
        aBytes = [adr]
        ljm.eWriteNameByteArray(handle, 'I2C_DATA_TX', numTX, aBytes)
        ljm.eWriteName(handle, 'I2C_GO', 1)  # Do the I2C communications.

        # Wait 5 ms for measurement
        time.sleep(5E-3)

        # ADC read: 0x00 (00000000)
        # Sending 1 byte, receiving 3 bytes
        # (24-bit binary, unsigned 32-bit integer)
        numTX = 1
        numRX = 3
        ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', numTX)
        ljm.eWriteName(handle, 'I2C_NUM_BYTES_RX', numRX)

        # Set the TX bytes
        aBytes = [0x00]
        ljm.eWriteNameByteArray(handle, 'I2C_DATA_TX', numTX, aBytes)
        ljm.eWriteName(handle, 'I2C_GO', 1)  # Do the I2C communications.

        # Read the RX bytes.
        # aBytes[0] and aBytes[1] will contain the data
        # NOTE: the sensor is big-endian (most significant bit aka MSB is first)
        aBytes = ljm.eReadNameByteArray(handle, 'I2C_DATA_RX', numRX)

        # Combine the 3 packets of 8-bits into a 24-bit binary number
        bin24 = f'{aBytes[0]:08b}{aBytes[1]:08b}{aBytes[2]:08b}'

        # Convert the 24-bit binary string into an (unsigned!) 32-bit integer
        # https://stackoverflow.com/a/8928256
        pt_uncomp[i] = int(bin24, 2)

    # Temporarily turning off overflow warning as we are accounting for mixed
    # int types
    old_settings = np.geterr()
    np.seterr(over='ignore')

    # Relative temperature difference
    # dT = D2 - (C5 * 2^8)
    dT = pt_uncomp[1] - (pt_cal[4] * 2**8)

    # Calculate temperature in deg C
    # T = 2000 + dT * (C6 / 2^23)
    T = 2000 + (dT * (pt_cal[5] / 2**23))

    # Pressure offset at actual temperature
    # OFF = (C2 * 2^17) + (C4 * dT) / 2^6
    p_off = (pt_cal[1] * 2**17) + (pt_cal[3] * dT) / 2**6

    # Sensitivty at actual temperature
    # SENS = (C1 * 2^16) + (C3 * dT) / 2^7
    p_sens = (pt_cal[0] * 2**16) + (pt_cal[2] * dT) / 2**7

    # Second order temperature corrections
    if T < 20*100:
        T2 = 3 * dT**2 / 2**33
        p_off2 = 61 * (T - 2000)**2 / 2**4
        p_sens2 = 29 * (T - 2000)**2 / 2**4

        if T < -15*100:
            p_off2 += (17 * (T + 1500)**2)
            p_sens2 += (9 * (T + 1500)**2)

    else:
        T2 = 5 * dT**2 / 2**38
        p_off2 = 0
        p_sens2 = 0

    # Apply corrections
    T -= T2
    p_off -= p_off2
    p_sens -= p_sens2

    # Calculate temperature compensated pressure
    # (10--1200mbar with 0.01mbar resolution)
    # P = ((D1 * SENS) / 2^21 - OFF) / 2^15
    p = (pt_uncomp[0] * (p_sens / 2**21) - p_off) / 2**15

    # Return to previous settings
    np.seterr(**old_settings);

    # Humidity address: 0x40 (1110110)
    ljm.eWriteName(handle, 'I2C_SLAVE_ADDRESS', 0x40)

    # Let's start with a reset command
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', 1)
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_RX', 0)

    # Set the TX bytes
    numBytes = 1

    # Reset: 0xFE (11111110)
    aBytes = [0xFE]
    ljm.eWriteNameByteArray(handle, 'I2C_DATA_TX', numBytes, aBytes)
    ljm.eWriteName(handle, 'I2C_GO', 1)

    # Set the TX/RX bytes
    numTX = 1
    numRX = 1
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', numTX)
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_RX', numRX)

    # Read in user register values: 0xE7 (11100111)
    aBytes = [0xE7]
    ljm.eWriteNameByteArray(handle, 'I2C_DATA_TX', numTX, aBytes)
    ljm.eWriteName(handle, 'I2C_GO', 1)

    # Read the RX bytes.
    # aBytes[0] will contain the data
    # NOTE: the sensor is big-endian (most significant bit aka MSB is first)
    aBytes = ljm.eReadNameByteArray(handle, 'I2C_DATA_RX', numRX)

    # Convert into 8-bit binary string
    # This is the default user register
    ureg = f'{aBytes[0]:08b}'

    # Write in options to the user register
    numTX = 2
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', numTX)

    # Write command: 0xE6 (11100110)
    # Use the read in register and change corresponding bits:
    #   - OSR = 2048 (bit 7 = 0, bit 0 = 1)
    #   - Battery state VDD > 2.5 V (bit 6: 0)
    #   - Reserved (bits 1, 3--5: 0, 0, 0, 0)
    #   - On-chip heater disabled (bit 2: 0)

    bit7 = 0
    bit6 = 0
    bit5 = ureg[2]
    bit4 = ureg[3]
    bit3 = ureg[4]
    bit2 = 0
    bit1 = ureg[6]
    bit0 = 1
    ureg2 = f'0b{bit7}{bit6}{bit5}{bit4}{bit3}{bit2}{bit1}{bit0}'
    aBytes = [0xE6, int(ureg2, 2)]
    ljm.eWriteNameByteArray(handle, 'I2C_DATA_TX', numTX, aBytes)
    ljm.eWriteName(handle, 'I2C_GO', 1)

    # Initiate relative humidity measurement (no hold master)
    numTX = 1
    numRX = 3
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_TX', numTX)
    ljm.eWriteName(handle, 'I2C_NUM_BYTES_RX', numRX)

    # Set the TX bytes. RH (hold master): 0xE5 (11100101)
    aBytes = [0xE5]
    ljm.eWriteNameByteArray(handle, 'I2C_DATA_TX', numTX, aBytes)
    ljm.eWriteName(handle, 'I2C_GO', 1)

    # Read the RX bytes.
    # 3 bytes: [Data (14 bits), Status (2 bits), Checksum (8 bits)]
    # NOTE: the sensor is big-endian (most significant bit aka MSB is first)
    aBytes = ljm.eReadNameByteArray(handle, 'I2C_DATA_RX', numRX)

    # Combine the 2 packets of 8-bits into a 16-bit binary number
    bin16 = f'{aBytes[0]:08b}{aBytes[1]:08b}'

    # Convert the 14-bit binary string into an (unsigned!) 16-bit integer
    # https://stackoverflow.com/a/8928256
    rh_digital = int(bin16, 2)

    # Convert digital value to real relative humidity
    # RH = - 600 + 12500 * D3 / 216
    rh = -600 + (12500 * (rh_digital / 2**16))

    # First-order temperature correction
    Tcoeff = -0.18
    rh += (20 - T/100)*Tcoeff

    # Turn off sensor
    ljm.eWriteName(handle, power, off)

    # Close handle
    ljm.close(handle)

    # Round off values
    T = np.round(T/100, 2)
    p = np.round(p/100, 2)
    rh = np.round(rh/100, 2)

    # Return as (temperature [degC], pressure [mbar], relative humidity [%])
    return (T, p, rh)


if __name__ == '__main__':
    use_sensor = 'SHT31'

    if use_sensor == 'MS8607':
        (T, p, rh) = readMS8607('2idlabjackt7.xray.aps.anl.gov')
        print(f'Temperature (deg C): {T}')
        print(f'Air pressure (mbar): {p}')
        print(f'Relative humidity (%): {rh}')
    elif use_sensor == 'SHT31':
        (T, rh) = readSHT31('164.54.113.87')
        print(f'Temperature (deg C): {T}')
        print(f'Relative humidity (%): {rh}')
