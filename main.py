import numpy as np
from sklearn.utils import resample
import paho.mqtt.client as mqtt
from collections import deque
import threading
import time


class Data:
    devices = {}
    queue = deque()
    newValues = threading.Event()


def on_connect(client, userdata, flags, rc):
    client.subscribe("dt/tracking/#")


def on_message(client: mqtt, userdata: Data, msg):
    topic = msg.topic
    if topic.endswith("rssi"):
        try:
            root, context, txDeviceName, type, rxDeviceName, measurement = topic.split("/")
            rssi = int(msg.payload)
        except ValueError as e:
            print('error type: ', type(e))
        try:
            userdata.devices[txDeviceName][rxDeviceName] = {'rssi': rssi, 'time': msg.timestamp}
        # if len > 3 add to deque
        #            rssi_values = [value['rssi'] for key, value in data.items() if value['time'] > 40885]
            # [value['rssi'] for key, value in data.devices[txDeviceName].items() if value['time'] > 72890 ]
            rssi_values = [value['rssi'] for key, value in data.devices[txDeviceName].items() if time.monotonic() - value['time'] < 120]
            if len(rssi_values) > 2:
                data.queue.append((txDeviceName,userdata.devices[txDeviceName]))
        except KeyError as e:
            userdata.devices[txDeviceName] = {rxDeviceName: {'rssi': rssi, 'time': msg.timestamp}}
        print(userdata)


# https://chat.openai.com/chat/35de85b8-9e81-44ab-8f5a-6a86583d92fc
def trilateration_particle_filter_confidence(points, distances, n_particles, confidence):
    # Initialize particles at random locations
    particles = np.random.rand(n_particles, 3)

    # Iterate until convergence
    for i in range(n_iterations):
        # Calculate the distance from each particle to each point
        distances_from_points = np.array([np.linalg.norm(particles - point, axis=1) for point in points])

        # Calculate the likelihood of each particle given the measurements
        likelihood = np.prod(np.exp(-0.5 * (distances_from_points - distances) ** 2), axis=0)

        # Normalize the likelihood
        likelihood /= np.sum(likelihood)

        # Resample particles based on their likelihood
        particles = resample(particles, weights=likelihood)

    # The estimated location is the mean of the particles
    x = np.mean(particles, axis=0)

    # Compute the standard deviation of the particles
    std_dev = np.std(particles, axis=0)
    lower_bound, upper_bound = stats.norm.interval(confidence, loc=x, scale=std_dev)
    return x, lower_bound, upper_bound


if __name__ == '__main__':
    try:
        data = Data()
        client = mqtt.Client(userdata=data)
        client.on_connect = on_connect
        client.on_message = on_message

        client.connect("192.168.0.163", 1883, 60)
        #    client.loop_forever()
        client.loop_start()

        while True:
            while len(data.queue) > 0:
                data.queue.pop()
            time.sleep(5)
        client.loop_stop()
    except SystemExit as e:
        client.loop_stop(0)
