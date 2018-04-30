# MIT License
#
# Copyright (c) 2017 Hordur K Heidarsson / USC RESL
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Adapted from https://github.com/uscresl/piksi_ros/blob/master/src/piksi_driver.py

from socket import *
from sbp.client.drivers.base_driver import BaseDriver
from sbp.client import Handler, Framer
from collections import deque
import threading

# Driver class for handling UDP connections for SBP
class UDPDriver(BaseDriver):
    def __init__(self, host, port):
        self.buf = deque()
        self.handle = socket(AF_INET, SOCK_DGRAM)
        self.handle.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        try:
            #self.handle.connect((host, port))
            self.handle.bind(("", port))
        except socket.error, msg:
            pass
        super(UDPDriver, self).__init__(self.handle)
        self._write_lock = threading.Lock()

    def read(self, size):
        if len(self.buf) < size:
            try:
                data, addr = self.handle.recvfrom(4096)
                if not data:
                    print "PIKSI UDP ERROR - no data from " + str(addr)
                for d in data:
                    self.buf.append(d)
            except socket.error, msg:
                print "PIKSI UDP ERROR " + str(msg)

        res = ''.join([self.buf.popleft() for i in xrange(min(size, len(self.buf)))])
        return res

    def flush(self):
        pass

    def write(self, s):
        return
        """
        Write wrapper.
        Parameters
        ----------
        s : bytes
        Bytes to write
        """
        try:
            self._write_lock.acquire()
            self.handle.sendall(s)
        except socket.error, msg:
            raise IOError
        finally:
            self._write_lock.release()



class UdpMulticaster:

    def __init__(self, broadcast, port):
        self.multicast_address = broadcast
        self.port = port
        self.socket = socket(AF_INET, SOCK_DGRAM)
        self.socket.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

    def sendPacket(self, data):
        self.socket.sendto(data, (self.multicast_address, self.port))



class SbpUdpMulticaster(UdpMulticaster):
    def __init__(self, broadcast, port):
        UdpMulticaster.__init__(self, broadcast, port)

    def sendSbpPacket(self, sbp_data):
        self.sendPacket(sbp_data.pack())


class SbpUdpMulticastReceiver:
    def __init__(self, port, ext_callback):
        self._callback = ext_callback
        self.driver = UDPDriver(' ', port)
        self.framer = Framer(self.driver.read, None, verbose=False)
        self.piksi = Handler(self.framer)
        self.piksi.add_callback(self.recv_callback)
        self.piksi.start()

    def recv_callback(self, msg, **metadata):
        self._callback(msg, **metadata)
