import sys, platform, struct
import zmq
import numpy, pygame

port = "5556"
if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)

# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

print "Collecting data from server..."
socket.connect ("tcp://localhost:%s" % port)

topicfilter = ""
socket.setsockopt(zmq.SUBSCRIBE, topicfilter)

pygame.mixer.init(12000, 16, 1, 1024)

while True:
    body = socket.recv()

    seq = struct.unpack('<I', body[0:4])[0]
    smeter = struct.unpack('>H', body[4:6])[0]
    rssi = (smeter & 0x0FFF) // 10 - 127

    messagedata = numpy.fromstring(body[6:], numpy.int16)
    pygame.mixer.Channel(0).queue(pygame.sndarray.make_sound(numpy.array(messagedata, numpy.int16)))

    sys.stdout.write('\r Sample Size: %-04d Block: %08x, RSSI: %-04d' % (len(messagedata), seq, rssi))
    sys.stdout.flush()
