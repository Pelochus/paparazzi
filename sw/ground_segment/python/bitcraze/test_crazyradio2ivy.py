# Extracted from:
# https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/cfbridge.py

"""
Bridge a Crazyflie connected to a Crazyradio to a local MAVLink port
Requires 'pip install cflib'

As the ESB protocol works using PTX and PRX (Primary Transmitter/Receiver)
modes. Thus, data is only received as a response to a sent packet.
So, we need to constantly poll the receivers for bidirectional communication.

@author: Dennis Shtatnov (densht@gmail.com)

Based off example code from crazyflie-lib-python/examples/read-eeprom.py
"""

# import struct
import logging
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket
# from cflib.crtp.crtpstack import CRTPPort

# Paparazzi UAV libs
from os import path, getenv
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
sys.path.append(PPRZ_HOME + "/var/lib/python")

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.pprz_transport import PprzTransport
from pprzlink.message import PprzMessage
import pprzlink.messages_xml_map as messages_xml_map

CRTP_PORT_PPRZLINK = 15
CRTP_PORT_MAVLINK = 8

# Debug output
logging.basicConfig(level=logging.DEBUG)

class RadioBridge:
    def __init__(self, link_uri, msg_class='telemetry', verbose=True):
        """ Initialize and run the example with the specified link_uri"""
        self.verbose = verbose

        # Ivy interface and stream parser
        self._ivy = IvyMessagesInterface("cf2ivy")
        self._transport = PprzTransport(msg_class)

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        # Connect some callbacks from the Crazyflie API
        self._cf.link_established.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True

        # Bind to all messages from ac_id
        def _forward_to_cf(ac_id, msg):
            try:
                data = self._transport.pack_pprz_msg(0, msg) # sender_id 0 = GCS
                for i in range(0, len(data), 30):
                    pk = CRTPPacket()
                    pk.port = CRTP_PORT_PPRZLINK
                    pk.data = data  # struct.pack('<fffH', roll, -pitch, yaw, thrust)
                    self._cf.send_packet(pk)
                if self.verbose:
                    print('Forward message', msg.name)
            except:
                if self.verbose:
                    print('Forward error for', ac_id)

        messages_datalink = messages_xml_map.get_msgs("datalink")
        
        for msg in messages_datalink:
            self._ivy.subscribe(_forward_to_cf, PprzMessage("datalink", msg))


    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        self._cf.packet_received.add_callback(self._got_packet)


    def _got_packet(self, pk):
        # print(pk.port) # Debug
        if (pk.port == CRTP_PORT_PPRZLINK):
            # print(pk.data) # Debug
            self._ivy.send("msg", 1)
            for c in pk.data:
                if self._transport.parse_byte(bytes([c])):
                    (sender_id, _, _, msg) = self._transport.unpack()
                    if self.verbose:
                        print("Got message {} from {}".format(msg.name, sender_id))
                    # Forward message to Ivy bus
                    if self.is_connected:
                        try:
                            self._ivy.send(msg, sender_id=sender_id)
                        except RuntimeError as e:
                            print("Runtime error {}".format(e))
                        except ValueError as e:
                            print("Invalid message error {}".format(e))


    def _forward(self, data):
        pk = CRTPPacket()
        pk.port = CRTP_PORT_PPRZLINK
        pk.data = data  # struct.pack('<fffH', roll, -pitch, yaw, thrust)
        self._cf.send_packet(pk)


    def shutdown(self):
        if self.verbose:
            print('closing cf2ivy interfaces')
        self._ivy.shutdown()


    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))


    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        print('[%d][%s]: %s' % (timestamp, logconf.name, data))


    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False


    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))


    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.radiodriver.set_retries_before_disconnect(1500)
    cflib.crtp.radiodriver.set_retries(3)
    cflib.crtp.init_drivers()

    if len(sys.argv) > 1:
        channel = str(sys.argv[1])
    else:
        channel = 80

    link_uri = 'radio://0/' + str(channel) + '/2M'
    le = RadioBridge(link_uri, verbose=True)

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    try:
        while le.is_connected:
            time.sleep(1)
    except KeyboardInterrupt:
        le.shutdown()
        sys.exit(1)
