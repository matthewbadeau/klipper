# Support Go!Temp USB Temperature Sensor
# Based on the LM75 module
#
#
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import struct
import os
import mcu

GOTEMP_REPORT_TIME = .8
# Temperature can be sampled at any time but the read aborts
# the current conversion. Conversion time is 300ms so make
# sure not to read too often.
GOTEMP_MIN_REPORT_TIME = .5

class GOTEMP:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self._usb_device = bytearray(config.get("usb_device").encode())
        self._mcu = mcu.get_printer_mcu(self.printer, config.get('sensor_mcu'))

        self.temp = self.min_temp = self.max_temp = 0.0

        self.report_time = config.getfloat('gotemp_report_time', GOTEMP_REPORT_TIME,
                                           minval=GOTEMP_MIN_REPORT_TIME)
        self.sample_timer = self.reactor.register_timer(self._sample_gotemp)
        self.printer.add_object("gotemp " + self.name, self)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        self._init_gotemp()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return self.report_time

    def _init_gotemp(self):
        try:
            usb_device = os.open(self._usb_device, os.O_RDONLY)
            rv = os.read(usb_device, 8)
            os.close(usb_device)
            logging.info("Go!Temp: Initial raw value %#x" % rv)
        except:
            pass

    def _sample_gotemp(self, eventtime):
        try:
            usb_device = os.open(self._usb_device, os.O_RDONLY)
            rv = os.read(usb_device, 8)
            os.close(usb_device)
            raw_parsed_pkt = list(struct.unpack("<BBHHH", rv))
            num_samples = raw_parsed_pkt.pop(0)
            seqno = raw_parsed_pkt.pop(0)

            parsed_samples = [sample/128.0 for sample in raw_parsed_pkt]
            avg_sample = sum(parsed_samples)/len(parsed_samples)

            self.temp = avg_sample
        except Exception:
            logging.exception("Go!Temp: Error reading data")
            self.temp = 0.0
            return self.reactor.NEVER

        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "Go!Temp temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp))

        measured_time = self.reactor.monotonic()
        self._callback(self._mcu.estimated_print_time(measured_time), self.temp)
        return measured_time + self.report_time

    def get_status(self, eventtime):
        return {
            'temperature': round(self.temp, 2),
        }


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("GOTEMP", GOTEMP)
