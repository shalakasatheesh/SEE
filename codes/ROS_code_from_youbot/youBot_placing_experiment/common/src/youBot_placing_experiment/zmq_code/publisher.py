import zmq
import time
import sys
import json
from pygments import highlight, lexers, formatters


# ZeroMQ Context
context = zmq.Context()

# Define the socket using the "Context"
sock = context.socket(zmq.PUB)
sock.bind("tcp://192.168.1.104:5680")

id = 0

while True:
    time.sleep(1)
    dictionary = {'marker_id': 0, "position": [1,1,1], "orientation": [2,2,2]}
    formatted_json = json.dumps(dictionary, indent=5)
    colorful_json = highlight(unicode(formatted_json, 'UTF-8'), lexers.JsonLexer(), formatters.TerminalFormatter())
    print(colorful_json)
    sock.send_json(colorful_json)

    print("Publishing message: " + str(id))
    id += 1