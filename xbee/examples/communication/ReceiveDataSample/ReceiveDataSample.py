
# Copyright 2017, Digi International Inc.
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from digi.xbee.devices import XBeeDevice
import sys
import subprocess

# TODO: Replace with the serial port where your local module is connected to. 
PORT = "/dev/tty.usbserial-DJ00I0E5"
# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 57600


def main():
    print(" +-----------------------------------------+")
    print(" | XBee Python Library Receive Data Sample |")
    print(" +-----------------------------------------+\n")

    try:
        device = setup_xbee()

        # device = XBeeDevice(PORT, BAUD_RATE)
        # device.open()

        def data_receive_callback(xbee_message):
            print("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                     xbee_message.data.decode()))

        device.add_data_received_callback(data_receive_callback)

        print("Waiting for data...\n")
        input()

    finally:
        if device is not None and device.is_open():
            device.close()


# Instantiates XBee device object
def setup_xbee():
    # Continues looking until connection is found
    # while (1):
    port_name = ""

    if sys.platform == "darwin":
        port_name = mac_xbee_port_name()
        print(port_name)
    elif sys.platform == "linux" or sys.platform == "linux2":
        port_name = "/dev/ttyUSB0"
    # TODO: figure out windows port name
    elif sys.platform == "win32":
        port_name = "COMS1"

    # Instantiate XBee device object.
    # xbee = XBeeDevice(port_name, 57600)
    xbee = XBeeDevice(PORT, 57600)
    print('\'' + port_name + '\'')
    print('\'' + PORT + '\'')

    xbee.open()

    print("set-up complete")

    return xbee 


# Looks in /dev directory for connected XBee serial port name on a macOS.
def mac_xbee_port_name():
    try:
        # System call to get port name of connected XBee radio
        port_name = subprocess.check_output(["ls", "/dev/"])

        i = port_name.index("tty.usbserial-")  # index in dev directory of port name
        return "/dev/" + port_name[i: i + 22]  # 22 is length of "tty.usbserial-" + 8-char port name

    except ValueError:
        raise ValueError("Value Error: \'tty.usbserial-\' not found in /dev")


if __name__ == '__main__':
    main()
