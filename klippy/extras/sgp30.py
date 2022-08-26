# Support for I2C based SGP30/SGP30A temperature sensors
#
# Copyright (C) 2022  Matthew Badeau <matthewbadeau@users.noreply.github.com>
# Datasheet used: https://sensirion.com/media/documents/984E0DD5/61644B8B/Sensirion_Gas_Sensors_Datasheet_SGP30.pdf
# Inspiration and code use from:
# Pimoroni: https://github.com/pimoroni/sgp30-python
# ladyada:  https://github.com/adafruit/Adafruit_CircuitPython_SGP30
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import struct
from . import bus

SGP30_CHIP_ADDR = 0x58
SGP30_I2C_SPEED = 100000
SGP30_FEATURE_SETS = (0x0020, 0x0022)
SGP30_CRC8_POLYNOMIAL = 0x31
SGP30_CRC8_INIT = 0xFF
SGP30_WORD_LEN = 2

SGP30_REGS = {
    'sgp30_soft_reset':  ([0x00, 0x06], 0, 0),                   # Soft reset
    'sgp30_iaq_init':  ([0x20, 0x03], 0, 0),                     # Init air quality
    'sgp30_measure_iaq':  ([0x20, 0x08], 0, 6),                  # Measure air quality
    'sgp30_get_iaq_baseline':  ([0x20, 0x15], 0, 6),             # Get air quality baseline
    'sgp30_set_iaq_baseline':  ([0x20, 0x1e], 6, 0),             # Set air quality baseline
    'sgp30_set_absolute_humidity':  ([0x20, 0x61], 3, 0),        # Set absolute humidity
    'sgp30_get_feature_set':  ([0x20, 0x2f], 0, 3),              # Get feature set
    'sgp30_measure_raw':  ([0x20, 0x50], 0, 6),                  # Measure raw signals
    'sgp30_get_tvoc_inceptive_baseline':  ([0x20, 0xb3], 0, 3),  # Get total VOC starting reference baseline
    'sgp30_set_tvoc_baseline':  ([0x20, 0x77], 3, 0),            # Set total VOC baseline
    'sgp30_serial_id': ([0x36, 0x82], 0, 3),                     # SGP30 Serial ID
}
SGP30_REPORT_TIME = .8

COMP_TH_SENSORS = [                                        # Compatible Temp & Humidity Sensors for compensation
    'BME280',
    'BME680',
    'BME688',
#    'DHT22',
    'HTU21D',
]


class SGP30:
    def __init__(self, config):
        self.debug = config.get('debug', False)
        self.registers = SGP30_REGS

        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(config, default_addr=SGP30_CHIP_ADDR,
                                           default_speed=SGP30_I2C_SPEED)
        self.mcu = self.i2c.get_mcu()

        self.report_time = config.getfloat('sgp30_report_time', SGP30_REPORT_TIME)
        self.comp_sensor_initialized = False
        self.init_temp = self.init_humidity = 0.

        self.init_time = 15  # Time in seconds
        self.tvocs_min = 0    # ppb
        self.co2eq_min = 400  # ppm

        self.max_sample_time = 0.25
        self.sample_timer = None
        # self.sample_timer = self.reactor.register_timer(self._sample_sgp30())
        logging.info("SGP30 starting")

        self.printer.add_object("sgp30 " + self.name, self)
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        self._init_sgp30()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_callback(self, cb):
        self._callback = cb

    def _sample_sgp30(self):
        # self.tvocs, self.co2eq = self.i2c.i2c_read(SGP30_REGS['sgp30_measure_iaq'][0], 6)
        value = self.i2c.i2c_read([self.registers['sgp30_measure_iaq'][0]], 6)
        print(value)

    def _init_sgp30(self):
        def _query_temphum_sensors():
            for sensor_type in COMP_TH_SENSORS:
                comp_sensor = self.printer.lookup_object('temperature_sensor sgp30_th_comp')
                if comp_sensor:
                    self.comp_sensor = comp_sensor
                    self.comp_sensor_type = sensor_type
                    self.init_temp = self.comp_sensor.sensor.temp
                    self.init_humidity = self.comp_sensor.sensor.humidity
                    self.comp_sensor_initialized = True
                    return
        if self.debug:
            logging.info('SGP30 init')

        if not self.comp_sensor_initialized:
            _query_temphum_sensors()
        logging.info("SGP30 initial soft reset")
        self.i2c.i2c_write(self.registers['sgp30_soft_reset'][0])
        # self.reactor.pause(self.reactor.monotonic() + 0.15)
        self.i2c.i2c_write(self.registers['sgp30_iaq_init'][0])
        # self.reactor.pause(self.reactor.monotonic() + 0.15)
        if self.init_temp != 0 and self.init_humidity != 0:
            self._send_command('sgp30_set_absolute_humidity', [self.init_temp, self.init_humidity])
            self.reactor.pause(self.reactor.monotonic() + 0.15)

        # Throw out the first samples while initializing
        for i in range(self.init_time):
            self.reactor.register_timer(self._initialization_timers(self.reactor.monotonic(), i))

        # Report Serial ID
        # Identifies chip and presence
        try:
            serial_id = self._get_unique_id
            logging.info("SGP30: Serial ID %#x" % serial_id)
        except:
            pass

    def _initialization_timers(self, measured_time, incrementer):
        rubbish_measurement = self._send_command('sgp30_measure_iaq')
        if self.debug:
            logging.info('SGP Init discarded measurement: {rubbish}'.format(rubbish=rubbish_measurement))
        return measured_time + incrementer

    def _get_unique_id(self):
        params = self.i2c.i2c_read(self.registers['sgp30_serial_id'][0])
        resp = bytearray(params['response'])
        rsid = resp[0] << 8
        rsid |= resp[1]
        checksum = resp[2]
        if self._calculate_crc(rsid) != checksum:
            logging.warning('SGP30: ERROR: Reading serial ID checksum')
        rsid = rsid >> 8
        return rsid

    def _send_command(self, reg_name, parameters=None):
        if parameters is None:
            parameters = []
        parameters = list(parameters)
        cmd, param_len, response_len = self.registers[reg_name]
        if len(parameters) != param_len:
            raise ValueError("{} requires {} parameters. {} supplied!".format(
                reg_name,
                param_len,
                len(parameters)
            ))

        parameters_out = [cmd]

        for i in range(len(parameters)):
            parameters_out.append(parameters[i])
            parameters_out.append(self._calculate_crc(parameters[i]))

        data_out = struct.pack('>H' + ('HB' * param_len), *parameters_out)
        print(data_out)

        msg_w = self.i2c.write(SGP30_CHIP_ADDR, data_out)
        self.reactor.pause(self.reactor.monotonic() + .025)

        if response_len > 0:
            # Each parameter is a word (2 bytes) followed by a CRC (1 byte)
            msg_r = self.i2c.i2c_read(SGP30_CHIP_ADDR, None, response_len * 3)

            buf = msg_r.buf[0:response_len * 3]

            response = struct.unpack(
                '>' + ('HB' * response_len),
                buf)

            verified = []
            for i in range(response_len):
                offset = i * 2
                value, crc = response[offset:offset + 2]
                if crc != self._calculate_crc(value):
                    raise RuntimeError("Invalid CRC in response from SGP30: {:02x} != {:02x}",
                                       crc,
                                       self._calculate_crc(value),
                                       buf)
                verified.append(value)
            return verified

    def _calculate_crc(self, data):
        crc = SGP30_CRC8_INIT
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc <<= 1
                if crc & 0x100:
                    crc ^= SGP30_CRC8_POLYNOMIAL
        return crc


def load_config_prefix(config):
    # Register sensor
    return SGP30(config)
