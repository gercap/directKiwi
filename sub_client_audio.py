import sys, struct, platform
import zmq
import numpy, pygame
from optparse import OptionParser


if __name__ == '__main__':

	parser = OptionParser()

	parser.add_option('-z',
					'--zmq_port',
					dest='zmq_port',
					type='int', default=0,
					help='ZMQ SUB port')

	(options, unused_args) = parser.parse_args()

	# Socket to talk to server
	context = zmq.Context()
	socket = context.socket(zmq.SUB)

	print "Collecting data from server..."
	socket.connect ("tcp://localhost:%s" % options.zmq_port)

	topicfilter = ""
	socket.setsockopt(zmq.SUBSCRIBE, topicfilter)

	if platform.system() == "Darwin":  # deal with MacOS X systems
		import scipy
		from scipy import signal
		pygame.init()
	else:
		pygame.mixer.init(12000, 16, 1, 1024)

	while True:
		body = socket.recv()

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
