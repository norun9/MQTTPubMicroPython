from machine import I2C, Pin
import time
import struct

# Constants
I2C_MASTER_SCL_IO = 21  # SCLピンの番号、必要に応じて調整
I2C_MASTER_SDA_IO = 22  # SDAピンの番号、必要に応じて調整
I2C_MASTER_FREQ_HZ = 400000  # I2Cクロックの周波数 I2C通信速度: 最大400 kHz
SCD41_SENSOR_ADDR = 0x62  # SCD41のI2Cアドレス

# I2C initialization
i2c = I2C(0, scl=Pin(I2C_MASTER_SCL_IO), sda=Pin(I2C_MASTER_SDA_IO), freq=I2C_MASTER_FREQ_HZ)

# CRC8 Polynomial and Initial Value
CRC8_POLYNOMIAL = 0x31
CRC8_INIT = 0xFF

scan_res = i2c.scan()
print([hex(x) for x in scan_res])

# see if we read the correct id for the chip
bmp180_i2c_addr = 0x77


def bmp180_read_chip_id(bus):
    bmp180_reg_chip_id = b'\xd0'

    bus.writeto(bmp180_i2c_addr, bmp180_reg_chip_id, False)
    chip_id = bus.readfrom(bmp180_i2c_addr, 1)

    if chip_id[0] == 0x55:
        print("chip id is 0x55, BMP180 detected")
    else:
        print(f"chip id is: {hex(chip_id[0])}, NOT BMP180!")


def bmp180_read_coefficients(bus) -> Bytes:
    bmp180_coef_reg_base = b'\xaa'
    bmp180_coef_size = 22

    bus.writeto(bmp180_i2c_addr, bmp180_coef_reg_base, False)
    coefs = bus.readfrom(bmp180_i2c_addr, bmp180_coef_size)

    print(f"bmp coefficients: {coefs=}")
    return coefs


def bmp180_perform_measurement(bus, command: Bytes, ms: int) -> Bytes:
    bmp180_reg_out_msb = b'\xf6'

    bus.writeto(bmp180_i2c_addr, command, True)
    time.sleep_ms(ms)

    bus.writeto(bmp180_i2c_addr, bmp180_reg_out_msb, False)
    out = bus.readfrom(bmp180_i2c_addr, 3)

    # print(f"raw output: {[hex(x) for x in out]}")
    return out


def bmp180_read_temperature(bus) -> int:
    bmp180_cmd_meas_temp = b'\xf4\x2e'

    return bmp180_perform_measurement(bus, bmp180_cmd_meas_temp, 5)


def bmp180_read_pressure(bus) -> int:
    bmp180_cmd_meas_temp = b'\xf4\xf4'

    return bmp180_perform_measurement(bus, bmp180_cmd_meas_temp, 26)


def compute(coef, raw_temp, raw_press):
    # -> Tuple[float, float]

    # this is horrible, but it is what the spec sheet says you should do
    # first, let's parse our coefficients
    # print("data computation")

    # int.from_bytes exists, but more limited to struct
    # UT = int.from_bytes(raw_temp, 'big', True)
    UT = struct.unpack_from(">h", raw_temp)[0]
    # Q what do we do with xlsb?
    # UP = struct.unpack_from(">h", raw_press)[0]
    # UP is.. special, time to shift things around
    oss = 3
    UP = raw_press[0] << 16 | raw_press[1] << 8 | raw_press[2]
    UP = UP >> (8 - oss)

    AC1 = struct.unpack_from(">h", coef)[0]
    AC2 = struct.unpack_from(">h", coef, 2)[0]
    AC3 = struct.unpack_from(">h", coef, 4)[0]
    AC4 = struct.unpack_from(">H", coef, 6)[0]
    AC5 = struct.unpack_from(">H", coef, 8)[0]
    AC6 = struct.unpack_from(">H", coef, 10)[0]
    B1 = struct.unpack_from(">h", coef, 12)[0]
    B2 = struct.unpack_from(">h", coef, 14)[0]
    MB = struct.unpack_from(">h", coef, 16)[0]
    MC = struct.unpack_from(">h", coef, 18)[0]
    MD = struct.unpack_from(">h", coef, 20)[0]

    # compute temperature
    X1 = (UT - AC6) * AC5 // 0x8000
    X2 = MC * 0x0800 // (X1 + MD)
    B5 = X1 + X2
    T = (B5 + 8) // 0x0010

    # compute pressure
    B6 = B5 - 4000
    X1 = (B2 * (B6 * B6 // (1 << 12))) // (1 < 11)
    X2 = AC2 * B6 // (1 << 11)
    X3 = X1 + X2
    B3 = (((AC1 * 4 + X3) << oss) + 2) // 4
    X1 = AC3 * B6 // (1 << 13)
    X2 = (B1 * (B6 * B6 // (1 << 12))) // (1 << 16)
    X3 = ((X1 + X2) + 2) // 4

    # unsigned longs here, check later
    B4 = AC4 * (X3 + 32768) // (1 << 15)
    B7 = (UP - B3) * (50000 >> 3)
    if B7 < 0x80000000:
        p = (B7 * 2) // B4
    else:
        p = (B7 // B4) * 2
    X1 = (p // 256) * (p // 256)
    X1 = (X1 * 3038) // (1 << 16)
    X2 = (-7357 * p) // (1 << 16)
    p = p + (X1 + X2 + 3791) // 16

    return T / 10, p / 100


def generate_crc(data):
    crc = CRC8_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ CRC8_POLYNOMIAL
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


def stop_periodic_measurements():
    i2c.writeto(SCD41_SENSOR_ADDR, bytes([0x3F, 0x86]))


def start_periodic_measurements():
    i2c.writeto(SCD41_SENSOR_ADDR, bytes([0x21, 0xB1]))


def get_data_ready_status():
    i2c.writeto(SCD41_SENSOR_ADDR, bytes([0xE4, 0xB8]))
    read_buf = i2c.readfrom(SCD41_SENSOR_ADDR, 3)
    answer = int.from_bytes(read_buf[:2], 'big')
    return (answer & 0x07FF) != 0


def read_measurement():
    # センサーに0xEC05というコマンドを送信します。このコマンドは、測定データを読み出すためのものです。
    i2c.writeto(SCD41_SENSOR_ADDR, bytes([0xEC, 0x05]))
    return i2c.readfrom(SCD41_SENSOR_ADDR, 9)  # Responseの欄の値すべてで合計9バイト読み取る


def is_data_crc_correct(data):
    for i in range(3):
        # CRCのデータを配列から抽出する
        expected_crc = data[3 * i + 2]
        # チェックサムアルゴリズムを利用する
        calculated_crc = generate_crc(data[3 * i:3 * i + 2])
        if expected_crc != calculated_crc:
            print(f"SCD41: CRC ERROR at word number {i}")
            return False
    return True


def calculate_and_show_data(raw):
    # -> Tuple[int, float, float]

    # co2は格納された値をそのまま利用できる
    co2 = int.from_bytes(raw[0:2], 'big')  # 0から1までの要素0x01f4
    # raw[2]はCRC of 0x7b(10111011 8bit=1byte)
    raw_temperature = int.from_bytes(raw[3:5], 'big')  # 3から4までの要素0x6667
    # raw[5]はCRC of 0xa2
    raw_humidity = int.from_bytes(raw[6:8], 'big')  # 6から7までの要素0x5eb9
    # raw[8]はCRC of 0x3c

    temperature = -45 + 175 * (raw_temperature / 65535.0)  # T = - 45 + 175 * word[1](=Temp) / 2^16
    humidity = 100 * (raw_humidity / 65535.0)  # RH = 100 * word[2](=RH) / 2^16

    return co2, temperature, humidity


def poll_sensor():
    if not get_data_ready_status():
        print("SCD41: No new data available")
        return None
    measurements = read_measurement()
    if is_data_crc_correct(measurements):
        return calculate_and_show_data(measurements)
    else:
        print("SCD41: CRC error!")
        return None


RPR_ADDR = 0x38


def rpr_system_control():
    i2c.writeto(RPR_ADDR, bytes([0x40, 0x80]))


def rpr_mode_control():
    i2c.writeto(RPR_ADDR, bytes([0x41, 0x8a]))


def rpr_als_control():
    i2c.writeto(RPR_ADDR, bytes([0x42, 0x02]))


def read_als_data():
    # レジスタからデータを読み取り
    i2c.writeto(RPR_ADDR, bytes([0x46, 0x01]))
    data = i2c.readfrom(RPR_ADDR, 2)
    als_value = (data[1] << 8) | data[0]
    return als_value


rpr_system_control()
rpr_mode_control()
rpr_als_control()

bmp180_read_chip_id(i2c)
coef = bmp180_read_coefficients(i2c)

stop_periodic_measurements()
time.sleep(1)  # Delay for 1 second
start_periodic_measurements()


def bmp180_read_data():
    raw_temp = bmp180_read_temperature(i2c)
    raw_press = bmp180_read_pressure(i2c)
    return compute(coef, raw_temp, raw_press)


def scd41_read_data():
    return poll_sensor()
