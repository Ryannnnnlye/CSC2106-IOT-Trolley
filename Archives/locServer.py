import time
import board
import busio
import digitalio
import adafruit_rfm9x
import statistics
import math
import matplotlib
import random
# from random import randrange
# import graph
# matplotlib.use('TkAgg')
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation

from paho.mqtt import client as mqtt_client
# import mqttPublisher


broker = 'broker.emqx.io'
port = 1883
topic = "/python/mqtt"
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'emqx'
password = 'public'
# client = mqttPublisher.connect_mqtt()

RADIO_FREQ_MHZ = 900.0  # set radio frequency
# digital io pins
CS = digitalio.DigitalInOut(board.CE1)
RESET = digitalio.DigitalInOut(board.D25)
# initialize spi bus
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
# initialize radio driver
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# radio parameters
rfm9x.tx_power = 23
rfm9x.enable_crc = True
rfm9x.ack_delay = 0

rfm9x.node = 4  # self address
rfm9x.destination = 1  # localized node address

# save previous values in case of total packet loss on one or more nodes
aPrev = 0
bPrev = 0
cPrev = 0
pathLoss = 2  # default to 2


def handlePacket(rawPacket: 'string') -> 'list, list, list':
    #   Takes a recieved packet string and splits the information along delimeters.
    #   Returns 3 lists for each set of node data.
    packetIndex = rawPacket.split(",")
    mode = ""
    # print(packetIndex)
    nodeAData = []
    nodeBData = []
    nodeCData = []
    fallDetect = False
    temp = 0
    sound = 0

    for index in packetIndex:
        # Changes read mode when a new dataset is detected
        if ("NodeA" in index):
            mode = "A"
        elif ("NodeB" in index):
            mode = "B"
        elif ("NodeC" in index):
            mode = "C"
        elif ("Fall" in index):
            fallDetect = True
        elif ("Overtemp" in index):
            temp = 2
        elif ("Undertemp" in index):
            temp = 1
        elif ("High Noise" in index):
            sound = 1
        elif ("EOF" in index):  # stops reading at EOF to prevent reading from invalid data
            break

        # add data points to the correct lists
        if (index.lstrip("-").isdigit()):  # minus sign is problematic with isdigit
            if (mode == "A"):
                nodeAData.append(int(index))
            elif (mode == "B"):
                nodeBData.append(int(index))
            elif (mode == "C"):
                nodeCData.append(int(index))

    # return lists
    return nodeAData, nodeBData, nodeCData, fallDetect, temp, sound


def calcDistance(nodeA: 'list', nodeB: 'list', nodeC: 'list', pathLoss) -> 'float, float, float':
    # set averages to arbitrary unreachable values for error checking
    avgNodeA = 1.1
    avgNodeB = 1.1
    avgNodeC = 1.1
    global aPrev
    global bPrev
    global cPrev

    if (bool(nodeA)):
        avgNodeA = statistics.mean(nodeA)
    if (bool(nodeB)):
        avgNodeB = statistics.mean(nodeB)
    if (bool(nodeC)):
        avgNodeC = statistics.mean(nodeC)

    # rssi parameters

    # rssi values at 1m
    AA = -51  # TODO tune this value
    AB = -55  # TODO tune this value
    AC = -86  # TODO tune this value

    # pathloss coeficient
    n = pathLoss  # TODO tune this value

    aDist = 0.0  # force floats
    bDist = 0.0
    cDist = 0.0

    if (avgNodeA != 1.1):  # check if average was populated
        # calculate distance with rssi
        aDist = math.pow(10, ((AA - avgNodeA) / (10 * n)))
        aPrev = aDist
    else:
        aDist = aPrev  # use most recent best guess of position

    if (avgNodeB != 1.1):  # check if average was populated
        # calculate distance with rssi
        bDist = math.pow(10, ((AB - avgNodeB) / (10 * n)))
    else:
        bDist = bPrev  # use most recent best guess of position

    if (avgNodeC != 1.1):  # check if average was populated
        # calculate distance with rssi
        cDist = math.pow(10, ((AC - avgNodeC) / (10 * n)))
    else:
        cDist = cPrev  # use most recent best guess of position

    print("A DISTANCE:" + str(aDist))
    print("B DISTANCE:" + str(bDist))
    print("C DISTANCE:" + str(cDist))

    return aDist, bDist, cDist

# r1^2 = x^2 + y^2 + z^2
# r2^2 = (x - x2)^2 + y^2 + z^2
# r3^2 = (x - x3)^2 + (y - y3)^2 + z^2

# x = (r1^2 - r2^2 + x2^2) / (2 * x2)
# y = (r1^2 - r3^2 + x3^2 + y3^2 - (2 * x3 * x))
# z = sqrt(r1^2 - x^2 - y^2)

# x1 0, y1 0
# x2 4, y2 0
# x3 3, y3 4


def trilateration(aDist: 'float', bDist: 'float', cDist: 'float') -> 'float, float':
    # static anchor node positions, in meters
    # TODO: update these:
    x2 = 1.5
    x3 = 1
    y3 = 1.5

    xPos = (math.pow(aDist, 2) - math.pow(bDist, 2) +
            math.pow(x2, 2)) / (2 * x2)
    yPos = (math.pow(aDist, 2) - math.pow(cDist, 2) +
            math.pow(x3, 2) + math.pow(y3, 2) - (2 * x3 * xPos)) / (2 * y3)
    # zPos = (math.sqrt(math.pow(aDist, 2) - math.pow(xPos, 2) - math.pow(yPos, 2))) #could produce a negative root

    return xPos, yPos


def estPathloss():
    rfm9x.destination = 3  # reference anchor node address (B)
    avgRSSI = 0

    A0 = -59  # TODO tune this
    d = 1.5  # TODO tune this (in meters)
    rfm9x.ack_retries = 1
    rfm9x.ack_wait = 0.1
    rfm9x.ack_delay = 0

    pathLossArr = []
    for i in range(5):
        if not rfm9x.send_with_ack(bytes("pLoss", "UTF-8")):
            print("pLoss failed")
        packet = rfm9x.receive(with_ack=True, with_header=True)

        if (packet is not None):
            pathLossArr.append(rfm9x.last_rssi)
            print("pathloss ping: " + str(rfm9x.last_rssi))
        # if not rfm9x.send_with_ack(bytes("ack", "UTF-8")):
        #     print("No Ack")

    rfm9x.destination = 1  # reset destination
    rfm9x.ack_retries = 2
    rfm9x.ack_wait = 0.2
    time.sleep(0.1)

    if (bool(pathLossArr)):
        avgRSSI = statistics.mean(pathLossArr)

    print("num " + str(-avgRSSI + A0))

    print("denom " + str(-10 * math.log10(d)))
    pathLoss = (-avgRSSI + A0) / (-10 * math.log10(d))

    return pathLoss


print("Waiting for messages...")

# client.loop_start
# client.loop_forever()


while True:
    # client.loop

    # attempt to recieve message
    packet = rfm9x.receive(with_ack=True, with_header=True)

    if packet is not None:
        packetData = ""
        # utf-8 decoding can cause problems with lossy comms like LoRa
        packetData = packet.decode('UTF-8', 'backslashreplace')

        # reset packet lists
        nodeA = []
        nodeB = []
        nodeC = []
        aDist = 0.0
        bDist = 0.0
        cDist = 0.0
        fallDetect = False
        temp = 0

        if ("Ping" not in packetData):  # if packet is not a simple ping it should be a data packet
            pathLoss = estPathloss()  # calculate pathloss coefficient
            print("Pathloss: " + str(pathLoss))
            # tell wearable pathloss is done
            if not rfm9x.send_with_ack(bytes("pLossDone", "UTF-8")):
                print("PlossDone No Ack")

            # process and parse packet string along delimeters, return into lists
            nodeA, nodeB, nodeC, fallDetect, temp, sound = handlePacket(
                packetData)
            # calculate distances from rssi data, requires the most fine tuning for accuracy
            aDist, bDist, cDist = calcDistance(nodeA, nodeB, nodeC, pathLoss)
            # calculate cartesian coordinates based on distances from nodes
            xPos, yPos = trilateration(aDist, bDist, cDist)
            # mqttPublisher.publishMsg(client, xPos, yPos, fallDetect, temp, sound)

        print(packetData)
        if not rfm9x.send_with_ack(bytes("Est Pathloss", "UTF-8")):
            print("No Ack")
        # else:
    # client.loop_forever()
