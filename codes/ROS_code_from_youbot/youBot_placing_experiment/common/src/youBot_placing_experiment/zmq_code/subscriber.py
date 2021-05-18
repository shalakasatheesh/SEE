import zmq, json
import numpy as np
import sys

if np.size(sys.argv) is not 2:
    print ('Provide ID of the desired maker as one argument to this script.\nPossible choices are 0 and 1.')
    sys.exit()
elif int(sys.argv[1]) is not 1 and int(sys.argv[1]) is not 0:
    print ('Please choose between 0 and 1.')
    sys.exit()

desired_marker = int(sys.argv[1])

# ZeroMQ Context
context = zmq.Context()

# Define the socket using the "Context"
sock = context.socket(zmq.SUB)

# Define subscription and messages with prefix to accept.
sock.setsockopt(zmq.SUBSCRIBE, "")
sock.connect("tcp://192.168.22.178:5680")


while True:
    message = sock.recv()    
    dictionary = json.loads(message)

    if (dictionary["marker_id"]  is desired_marker):
        print message + "\n\n\n"
    elif dictionary["marker_id"] is 1:
        continue
    
