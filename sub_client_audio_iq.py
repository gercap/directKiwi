import sys, struct, platform
import zmq
import numpy, pygame
from optparse import OptionParser

import matplotlib.pyplot as plots
from scipy import signal as sg

if __name__ == '__main__':

	parser = OptionParser()

	parser.add_option('-z',
					dest='zmq_port',
					type='int', default=0,
					help='ZMQ SUB port')
	parser.add_option('-s',
					dest='source_type',
					type='string', default="audio",
					help='audio or iq')


	(options, unused_args) = parser.parse_args()

	# Socket to talk to server
	context = zmq.Context()
	socket = context.socket(zmq.SUB)

	print "Collecting data from server..."
	socket.connect ("tcp://localhost:%s" % options.zmq_port)

	topicfilter = options.source_type
	socket.setsockopt(zmq.SUBSCRIBE, topicfilter)


	if topicfilter == 'audio':
		print topicfilter, "mode"
		if platform.system() == "Darwin":  # deal with MacOS X systems
			import scipy
			from scipy import signal
			pygame.init()
		else:
			pygame.mixer.init(12000, 16, 1, 1024)

		while True:
			pack = socket.recv()
			body = pack[len(topicfilter)+1:]
			
			seq = struct.unpack('<I', body[0:4])[0]
			smeter = struct.unpack('>H', body[4:6])[0]
			rssi = (smeter & 0x0FFF) // 10 - 127

			messagedata = numpy.fromstring(body[6:], numpy.int16)

			if platform.system() == "Darwin":  # deal with MacOS X systems
				mono = scipy.signal.resample_poly(numpy.int16(messagedata), 147, 40 * 2)
				stereo = numpy.empty([len(mono), 2], dtype=numpy.int16)
				for i in range(len(mono)):
					stereo[i][0] = numpy.int16(mono[i]);
					stereo[i][1] = numpy.int16(mono[i]);
				pygame.mixer.Channel(0).queue(pygame.sndarray.make_sound(stereo))
			else:
				pygame.mixer.Channel(0).queue(pygame.sndarray.make_sound(numpy.array(messagedata, numpy.int16)))

			sys.stdout.write('\r Sample Size: %-04d Block: %08x, RSSI: %-04d' % (len(messagedata), seq, rssi))
			sys.stdout.flush()

	elif topicfilter == 'iq':
		print topicfilter, "mode"

		plots.figure(1)
		plots.show(False)

		for a in range(100): #while True:

			pack = socket.recv()
			body = pack[len(topicfilter)+1:]
			
			seq = struct.unpack('<I', body[0:4])[0]
			smeter = struct.unpack('>H', body[4:6])[0]
			rssi = (smeter & 0x0FFF) // 10 - 127

			samples = numpy.fromstring(body[6:], numpy.complex64)
			
			welch_axis, psd_welch = sg.welch(samples, window='flattop', fs = 12000, nperseg= 1024, nfft = 4096)
			psd_welch_aligned = numpy.fft.fftshift(psd_welch)
			welch_axis_aligned = numpy.fft.fftshift(welch_axis)

			plots.figure(1)
			plots.figure(1).clear()

			plots.plot(10*numpy.log10(psd_welch_aligned))
			plots.ylabel('PSD [dB/Hz]')
			plots.title('FFT method Welch', fontsize=12)
			plots.pause(0.05)
			plots.draw()

			sys.stdout.write('\r Sample Size: %-04d Block: %08x, RSSI: %-04d' % (len(samples), seq, rssi))
			sys.stdout.flush()

	else:
		print "source type unavailable"
