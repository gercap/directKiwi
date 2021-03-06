#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Gnuradio Interface
# Generated: Sun Dec 16 21:46:31 2018
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import analog
from gnuradio import audio
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import qtgui
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import pykiwisdr
import sip
import sys
from gnuradio import qtgui


class gnuradio_interface(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Gnuradio Interface")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Gnuradio Interface")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "gnuradio_interface")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())


        ##################################################
        # Variables
        ##################################################
        self.vlen = vlen = 1024
        self.samp_rate = samp_rate = 12000

        ##################################################
        # Blocks
        ##################################################
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
        	1024, #size
        	firdes.WIN_BLACKMAN_hARRIS, #wintype
        	0, #fc
        	samp_rate, #bw
        	"", #name
        	1 #number of inputs
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis(-140, 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, 0.0, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(True)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(True)

        if not True:
          self.qtgui_freq_sink_x_0.disable_legend()

        if "complex" == "float" or "complex" == "msg_float":
          self.qtgui_freq_sink_x_0.set_plot_pos_half(not True)

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["green", "blue", "red", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_freq_sink_x_0_win)
        self.pykiwisdr_kiwi_zmq_0 = pykiwisdr.kiwi_zmq(address='127.0.0.1', port='56077', zmq_filter='iq')
        self.blocks_tag_debug_0 = blocks.tag_debug(gr.sizeof_gr_complex*1, 'RSSI', "rssi"); self.blocks_tag_debug_0.set_display(True)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vff((0.2, ))
        self.audio_sink_0 = audio.sink(samp_rate/2, '', True)
        self.analog_feedforward_agc_cc_0 = analog.feedforward_agc_cc(1024, 1.0)
        self.analog_am_demod_cf_0 = analog.am_demod_cf(
        	channel_rate=samp_rate,
        	audio_decim=2,
        	audio_pass=3000,
        	audio_stop=4000,
        )



        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_am_demod_cf_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.analog_feedforward_agc_cc_0, 0), (self.analog_am_demod_cf_0, 0))
        self.connect((self.analog_feedforward_agc_cc_0, 0), (self.blocks_tag_debug_0, 0))
        self.connect((self.analog_feedforward_agc_cc_0, 0), (self.qtgui_freq_sink_x_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.audio_sink_0, 0))
        self.connect((self.pykiwisdr_kiwi_zmq_0, 0), (self.analog_feedforward_agc_cc_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "gnuradio_interface")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_vlen(self):
        return self.vlen

    def set_vlen(self, vlen):
        self.vlen = vlen

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate)


def main(top_block_cls=gnuradio_interface, options=None):

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
