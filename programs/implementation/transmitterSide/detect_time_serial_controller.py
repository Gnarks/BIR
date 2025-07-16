import time
from colors import bcolors
import signal
import socket
import serial
from serial import SerialException
from serial.tools import list_ports
import sys
import threading

# SERIAL_TO_ID is the dictionnary between serial and device id
from serials_list import SERIAL_TO_ID


class SerialController:
    def __init__(self, port, baudrate, device_id, frequency, cycles, bw, sf):
        self.ser = serial.Serial(port, baudrate)
        self.port = port

        # reset serial buffer
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.flush()

        # arduino settings
        self.frequency = frequency
        self.cycles = cycles
        self.bw = bw
        self.sf = sf
        self.device_id = device_id

        # flags
        self.initFlag = 0x7A.to_bytes(1, "big")
        self.sendFlag = 0x7B.to_bytes(1, "big")
        self.resetFlag = 0x7C.to_bytes(1, "big")
        self.responseFlag = 0x7D.to_bytes(1, "big")
        self.endFlag = 0x7F.to_bytes(1, "big")

        self.timeout = 5
        self.response_timeout_limit = 5
        self.skip = False
        self.ser.write_timeout = self.timeout
        self.ser.timeout = self.timeout

    def write(self, data):
        try:
            return self.ser.write(data)
        except SerialException:
            print(
                f"{bcolors.FAIL}Error reading from port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}",
                file=sys.stderr,
            )
            self.skip = True
            raise BufferError("unreachable device")
            return b""
        except OSError:
            print(
                f"{bcolors.WARNING}I/O error on port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
            )
            self.skip = True
            raise BufferError("unreachable device")

    def read(self):
        try:
            if self.ser.in_waiting == 0:
                return b""
            if self.skip:
                return b""
            return self.ser.read()
        except SerialException:
            print(
                f"{bcolors.FAIL}Error reading from port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}",
                file=sys.stderr,
            )
            self.skip = True
            raise BufferError("unreachable device")
            return b""
        except OSError:
            print(
                f"{bcolors.WARNING}Reading I/O error on port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
            )
            self.skip = True

    def send_packet(self):
        print(
            f"{bcolors.OKGREEN} Device {self.device_id} start Send at {time.time()} {bcolors.ENDC}"
        )
        self.write(self.sendFlag)
        self.wait_for_response_flag(self.remove)
        print(
            f"{bcolors.OKGREEN} Device {self.device_id} end Send at {time.time()} {bcolors.ENDC}"
        )

    def wait_for_response_flag(self, timeout_func=None):
        signal.signal(signal.SIGALRM, timeout_func)
        signal.alarm(self.timeout)
        if self.skip:
            return
        buffer_ = b""
        while (r := self.read()) != self.responseFlag:
            if self.skip:
                signal.alarm(0)
                return
            buffer_ += r
            if r == b"\n":
                print(buffer_)
                buffer_ = b""
        print(
            f"{bcolors.OKGREEN}Response flag received from {self.device_id}{bcolors.ENDC}"
        )
        signal.alarm(0)

    def reset(self):
        print(
            f"{bcolors.WARNING}Resetting device {self.device_id} on port {self.port} {bcolors.ENDC}"
        )
        self.write(self.resetFlag)

    def init_params(self):
        self.write(self.initFlag)
        self.write(
            f"{self.frequency},{self.cycles},{self.bw},{self.sf},{self.device_id}\n".encode()
        )

        self.wait_for_response_flag(self.init_timeout)

    def init_timeout(self, signum, frame):
        print(f"{bcolors.WARNING} Timeout, resending Init Flag {bcolors.ENDC}")
        self.init_params()
        self.wait_for_response_flag(self.panic)

    def remove(self):
        raise BufferError("unreachable device")


class Scheduler:
    def __init__(
        self, HOST, PORT, frequency, sf, bw, phase, period, cycles, number_of_devices
    ):
        self.controller_list = self.get_serial_controllers(
            number_of_devices, frequency, cycles, bw, sf
        )

        self.device_list = [x.device_id for x in self.controller_list]

        self.current_controller_index = 0
        self.current_controller = self.controller_list[0]
        self.cycles = cycles

        # variable list to send to receiver : in order :
        # frequency(without 1e6), sf, bw (without 1e3), device_list, starting_time, period, cycles
        try:
            self.initiate_all_devices(self.controller_list)

            self.starting_time = time.time() + phase

            var_list = [
                frequency,
                sf,
                bw,
                self.device_list,
                self.starting_time,
                period,
                cycles,
            ]

            self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.conn.connect((HOST, PORT))
            self.transmit_all_vars_to_socket(self.conn, var_list)

            self.scheduleSending(period, cycles)

        except Exception as e:
            for serial_controller in self.controller_list:
                serial_controller.reset()
            raise e
        except KeyboardInterrupt as e:
            self.transmit_var(self.conn, "close")
            self.conn.close()
            for serial_controller in self.controller_list:
                serial_controller.reset()
            raise e

    def initiate_all_devices(self, controller_list):
        for controller in controller_list:
            controller.init_params()

    def transmit_var(self, s, var):
        sep = "\n"
        val = str(var) + sep
        s.send(val.encode("utf-8"))
        print(f"sent {val}")

    def transmit_all_vars_to_socket(self, s, var_list):
        print(f"started transmitting params at {time.time()}")
        for var in var_list:
            self.transmit_var(s, var)
        print(f"finished transmitting params at {time.time()}")

    def get_serial_controllers(self, number_of_devices, frequency, cycles, bw, sf):
        device_list = list(list_ports.grep("ACM"))
        SERIAL_PORT_LIST = [(port.device, port.serial_number) for port in device_list]

        if len(device_list) == 0 or len(device_list) != number_of_devices:
            raise Exception(
                f"{len(device_list)} devices detected but {number_of_devices} wanted"
            )

        controller_list = []

        # sort by device_id
        SERIAL_PORT_LIST.sort(key=lambda x: int(SERIAL_TO_ID[x[1]]))
        for device_port, device_serial_number in SERIAL_PORT_LIST:
            device_id = SERIAL_TO_ID[device_serial_number]
            print(f"device_id {device_id} connected ")
            serial_controller = SerialController(
                device_port, 115200, int(device_id), frequency, cycles, bw, sf
            )
            controller_list.append(serial_controller)

        print(f"finished sending at {time.time()}")
        return controller_list

    def wait_until(self, sleep):
        print(f"waiting to {sleep} from {time.time()}")
        while time.time() < sleep:
            continue

    def scheduleSending(self, period, cycles):
        print(f"starting time : {self.starting_time}")
        self.current_cycle = 0

        while self.starting_time > time.time():
            continue

        sleep = self.starting_time

        while self.current_cycle < cycles:
            try:
                # if everything goes to plan : send packet then sleep for next
                self.current_controller.send_packet()
                sleep += period
                self.wait_until(sleep)
                self.change_to_next_controller()
            except BufferError as b:
                # if a device couldn't send a packet
                # sleep to new starting time set between Rx and Tx
                print("================================")
                print(str(b))
                print("================================")
                sleep = self.handle_device_crash()
                self.wait_until(sleep)
            except KeyboardInterrupt as e:
                raise e

        if self.current_cycle == self.cycles:
            print("all cycles are done, waiting 2 period before closing the socket")
            self.wait_until(time.time() + 2 * period)
            self.transmit_var(self.conn, "close")
            self.conn.close()

    def change_to_next_controller(self):
        # change device
        print(f"moved from {self.current_controller.device_id} ", end="")
        changeCycle = self.current_controller_index + 1 == len(self.controller_list)
        self.current_controller_index = (self.current_controller_index + 1) % len(
            self.controller_list
        )
        self.current_controller = self.controller_list[self.current_controller_index]

        print(f"to {self.current_controller.device_id}")
        # if the last device in the list
        if changeCycle:
            self.current_cycle += 1
            print(f"next cycle :{self.current_cycle}")

        print()

    def handle_device_crash(self):
        print(
            f"device {self.current_controller.device_id} crashed ! removing it from the list"
        )
        # first tell the Rx to remove a device
        self.transmit_var(self.conn, "remove")
        # remove it from the list and take the next
        to_remove = self.current_controller
        self.controller_list.remove(to_remove)

        if self.current_controller_index >= len(self.controller_list):
            self.current_cycle += 1
            self.current_controller_index = self.current_controller_index % len(
                self.controller_list
            )

        self.current_controller = self.controller_list[self.current_controller_index]

        # send the device id to remove
        self.transmit_var(self.conn, to_remove.device_id)

        restart_time = time.time() + 1
        # tell to sleep to actual time + 1s
        self.transmit_var(self.conn, restart_time)

        # restart with current index
        self.transmit_var(self.conn, self.current_controller_index)

        return restart_time


if __name__ == "__main__":
    if "-d" in sys.argv:
        HOST = "127.0.0.1"
        PORT = 12345
        frequency = 868
        sf = 7
        bw = 125
        phase = 3
        period = 1
        cycles = 10
        number_of_devices = int(input("number of devices (default=1): ").strip() or "1")
    else:
        # connect to receiver
        HOST = input("ip of receiver (default=127.0.0.1): ").strip() or "127.0.0.1"
        PORT = int(input("port (default=12345): ").strip() or "12345")

        frequency = float(
            input("frequency in MHz (default=868.1): ").strip() or "868.1"
        )
        sf = int(input("sf (default=7): ").strip() or "7")
        bw = float(input("bw in KHz (default=125): ").strip() or "125")
        phase = float(
            input(
                "time to wait before start of transmission in seconds (default=2): "
            ).strip()
            or "2"
        )
        period = float(
            input(
                "period or interval between transmissions in seconds (default=1): "
            ).strip()
            or "1"
        )
        cycles = int(
            input("#transmissions per device (cycle) (default=1000): ").strip()
            or "1000"
        )
        number_of_devices = int(input("number of devices (default=1): ").strip() or "1")

    Scheduler(HOST, PORT, frequency, sf, bw, phase, period, cycles, number_of_devices)
