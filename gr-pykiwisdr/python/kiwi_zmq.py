#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2018 gercap.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import numpy as np
from gnuradio import gr
import zmq, struct, sys, pmt

class kiwi_zmq(gr.sync_block):
    """
    docstring for block kiwi_zmq
    """
    def __init__(self, address, port, zmq_filter):
        gr.sync_block.__init__(self,
            name="kiwi_zmq",
            in_sig=None,
            out_sig=[np.complex64])

        # Socket to talk to server
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)

        print "Collecting data from server..."
        self.socket.connect ("tcp://%s:%s" % (address, port))

        self.topicfilter = zmq_filter
        self.socket.setsockopt(zmq.SUBSCRIBE, self.topicfilter)

    def work(self, input_items, output_items):
        out = output_items[0]
        
        pack = self.socket.recv()
        body = pack[len(self.topicfilter)+1:]
        
        seq = struct.unpack('<I', body[0:4])[0]
        smeter = struct.unpack('>H', body[4:6])[0]
        rssi = (smeter & 0x0FFF) // 10 - 127

        complex_samples = np.fromstring(body[6:], np.complex64)
        count = len(complex_samples)

        item_index = 0 #which output item gets the tag?
        offset = self.nitems_written(0) + item_index

        #write at tag to output port 0 with given absolute item offset
        self.add_item_tag(0, offset, pmt.string_to_symbol("rssi"), pmt.string_to_symbol(str(rssi)))

        for i in range(0, count):
            out[i] = complex_samples[i]
        return count

