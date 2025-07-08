import time
from colors import bcolors
import signal
import socket
import serial
from serial import SerialException, SerialTimeoutException
from serial.tools import list_ports
import sys

# SERIAL_TO_ID is the dictionnary between serial and device id
from serials_list import SERIAL_TO_ID

class SerialController:
    def __init__(self, port, 
                 baudrate, 
                 device_id,
                 frequency,
                 cycles,
                 bw,
                 sf):
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
        try :
            return self.ser.write(data)
        except SerialException:
            print(
                f"{bcolors.FAIL}Error reading from port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}",
                file=sys.stderr,
            )
            self.skip = True
            return b""
        except OSError:
            print(
                f"{bcolors.WARNING}I/O error on port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
            )
            self.skip = True

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
            return b""
        except OSError:
            print(
                f"{bcolors.WARNING}I/O error on port {self.port} : DEVICE {self.device_id}{bcolors.ENDC}"
            )
            self.skip = True

    # the reset from error handling
    def reset_error(self, signum, frame):
        print(f"{bcolors.FAIL}Error reseting of device {self.device_id} on port {self.port}{bcolors.ENDC}")
        self.reset()

    def send_packet(self):
        print(f"{bcolors.OKGREEN} Device {self.device_id} start Send at {time.time()} {bcolors.ENDC}")
        self.write(self.sendFlag)
        self.wait_for_response_flag(self.reset_error);
        print(f"{bcolors.OKGREEN} Device {self.device_id} end Send at {time.time()} {bcolors.ENDC}")
        

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
        print(f"{bcolors.OKGREEN}Response flag received from {self.device_id}{bcolors.ENDC}")
        signal.alarm(0)

    def reset(self):
        print(f"{bcolors.WARNING}Resetting device {self.device_id} on port {self.port} {bcolors.ENDC}")
        self.write(self.resetFlag)
        self.wait_for_response_flag(self.reset_error)

    # TODO add freq and cycles
    def init_params(self):
        self.write(self.initFlag)
        self.write(f"{self.frequency},{self.cycles},{self.bw},{self.sf},{self.device_id}\n".encode())

        self.wait_for_response_flag(self.init_timeout)

    def init_timeout(self, signum, frame):
        print(f"{bcolors.WARNING} Timeout, resending Init Flag {bcolors.ENDC}")
        self.init_params()
        self.wait_for_response_flag(self.panic)

    def panic(self, signum, frame):
        signal.signal(signal.SIGALRM, panic)
        signal.alarm(self.timeout)
        self.ser.close()
        signal.alarm(0)




def transmit_var(s, var):
    sep ='\n'
    val = str(var) + sep 
    s.send(val.encode("utf-8"))
    print(f"sent {val}")


def transmit_all_vars_to_socket(s, var_list):

    print(f'started transmitting params at {time.perf_counter()}')
    for var in var_list:
        transmit_var(s, var)
    print(f'finished transmitting params at {time.perf_counter()}')

def get_serial_controllers(number_of_devices,frequency, cycles, bw, sf):
    device_list = list(list_ports.grep("ACM"))
    SERIAL_PORT_LIST = [(port.device, port.serial_number) for port in device_list]

    if len(device_list) == 0 or len(device_list) != number_of_devices:
        raise Exception(f"{len(device_list)} devices detected but {number_of_devices} wanted")


    controller_list =[]

    # sort by device_id 
    SERIAL_PORT_LIST.sort(key=lambda x: int(SERIAL_TO_ID[x[1]]))
    for (device_port, device_serial_number) in SERIAL_PORT_LIST:
        device_id = SERIAL_TO_ID[device_serial_number]
        print(f"device_id {device_id} connected ")
        serial_controller = SerialController(device_port,115200, int(device_id), frequency, cycles, bw, sf)
        controller_list.append(serial_controller)

    print(f"finished sending at {time.perf_counter()}")
    return controller_list

def wait_until(sleep):
    print(f"waiting to {sleep} from {time.perf_counter()}")
    while (time.perf_counter() <  sleep ):
        continue

def initiate_all_devices(controller_list):
    print(f'started transmitting params at {time.perf_counter()}')
    for controller in controller_list:
        controller.init_params()

def scheduleSending(controller_list,starting_time, period, cycles):
    print(f"starting time : {starting_time}")
    current_cycle = 0

    # use time.time for absolute tim
    while starting_time > time.time():
        continue

    # go back to local time for more precision
    sleep = time.perf_counter() + period

    while current_cycle < cycles:
        for controller in controller_list:
            controller.send_packet()
            print(f"device {controller.device_id} send its {current_cycle} packet")
            wait_until(sleep)
            sleep += period

        current_cycle+=1


if __name__ == "__main__":

    # for dev only TODO delete
    if "-d" in sys.argv:
        HOST = "192.168.43.150"
        PORT = 12345
        frequency =868
        sf = 7
        bw = 125
        phase = 4
        period = 0.5
        cycles = 10
        number_of_devices = int(input('number of devices (default=1): ').strip() or "1")
    else :
        # connect to receiver
        HOST = input('ip of receiver (default=127.0.0.1): ').strip() or "127.0.0.1"
        PORT = int(input('port (default=12345): ').strip() or "12345")

        frequency = float(input('frequency in MHz (default=868.1): ').strip() or "868.1")
        sf = int(input('sf (default=7): ').strip() or "7")
        bw = float(input('bw in KHz (default=125): ').strip() or "125")
        phase = float(input('time to wait before start of transmission in seconds (default=2): ').strip() or "2")
        period = float(input('period or interval between transmissions in seconds (default=1): ').strip() or "1")
        cycles = int(input('#transmissions per device (cycle) (default=1000): ').strip() or "1000")
        number_of_devices = int(input('number of devices (default=1): ').strip() or "1")

    starting_time = time.time() + phase 

    controller_list = get_serial_controllers(number_of_devices,frequency, cycles, bw, sf)

    device_list = [x.device_id for x in controller_list]

    # variable list to send to receiver : in order :
    # frequency(without 1e6), sf, bw (without 1e3), device_list, starting_time, period, cycles
    var_list = [frequency, sf, bw, device_list, starting_time - period/2, period, cycles]

    try : 
        initiate_all_devices(controller_list)

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST,PORT))
        transmit_all_vars_to_socket(sock, var_list)

        scheduleSending(controller_list,starting_time, period, cycles)

    except:
        for serial_controller in controller_list:
            # Send interrupt flag to the receiver
            serial_controller.reset()

    # TODO later on don't close and thread it
    sock.close()



