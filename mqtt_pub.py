"""
import paho.mqtt.client as mqtt
import sys

broker = sys.argv[1]
topic = sys.argv[2]
message = sys.argv[3]

client = mqtt.Client()

try:
    client.connect(broker, 1883, 60)

    client.publish(topic, message)

    client.disconnect()
except:
    print "Something bad happend"
    sys.exit(1)
"""

import paho.mqtt.publish as publish
import sys

broker = sys.argv[1]
topic = sys.argv[2]
message = sys.argv[3]

try:
    publish.single(topic, message, hostname=broker)
except:
    print "Something bad happend"
    sys.exit(1)