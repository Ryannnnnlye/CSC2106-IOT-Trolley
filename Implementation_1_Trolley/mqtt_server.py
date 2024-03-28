

import json
import base64
import paho.mqtt.client as mqtt
from datetime import datetime

# Type the IP of your server
mqtt_ip = 'au1.cloud.thethings.network' 
mqtt_port = 1883
# Type your username and password. If the private MQTT server does not require username and password, commend the lines
mqtt_username = '12346969@ttn'
mqtt_password = 'NNSXS.CLMU2Z37X7EENC65R53442RNJ6IQC7ZXPAGJQFA.GMDK6KGO7EF6KUUFSIEP3O5AA7YAELVMTGIRQXFYSRUHK5EX6GVA'
# Replace the "id" with the id of your application and "eui" with your device eui
mqtt_rx_topic = 'v3/12346969@ttn/devices/eui-70b3d57ed0066002/up'

# Convert string to hexadecimal
def str_to_hex(s):
	return r"\x"+r'\x'.join([hex(ord(c)).replace('0x', '') for c in s])
# Once subscribed to the message, call back this method
def on_message(mqttc, obj, msg):
	on_print_rak_node_info(msg.payload)
# Print the subscribed node information
def on_print_node_rx_info(json_rx):
	try:
		devEUI = json_rx['devEUI']
		applicationID = json_rx['applicationID']
		applicationName = json_rx['applicationName']
		deviceName = json_rx['deviceName']
		timestamp = json_rx['timestamp']
		fCnt = json_rx['fCnt']
		fPort = json_rx['fPort']
		data = json_rx['data']
		data_hex = str_to_hex(base64.b64decode(data).decode("latin-1"))
		# Convert the timestamp to local time
		str_local_time = datetime.fromtimestamp(timestamp)
		print('---------------- devEUI:[%s] rxpk info -------------------' % devEUI)
		print('+\t applicationName:\t%s' % applicationName)
		print('+\t applicationID:\t\t%s' % applicationID)
		print('+\t deviceName:\t\t%s' % deviceName)
		print('+\t datetime:\t\t%s' % str_local_time)
		print('+\t fCnt:\t\t\t%d' % fCnt)
		print('+\t fPort:\t\t\t%d' % fPort)
		print('+\t data:\t\t\t%s' % data)
		print('+\t data_hex:\t\t%s' % data_hex)
		print('----------------------------------------------------------')
	
	except Exception as e:
		print(e)
	finally:
		pass

# After subscribing to the node's data, send the "Hello RAKwireless" string to the node
def on_print_rak_node_info(payload):
    json_str = payload.decode()
    try:
        json_rx = json.loads(json_str)
        on_print_node_rx_info(json_rx)
        if "uplink_message" in json_rx and "decoded_payload" in json_rx["uplink_message"]:
            payload = json_rx["uplink_message"]["decoded_payload"]
            if payload:
                print("decoded_payload:", payload)
            else:
                print("no payload")
        else:
            print("No decoded payload available")
 
    except Exception as e:
        print("An error occurred:", e)
	
mqttc = mqtt.Client(client_id="python_test")

mqttc.on_message = on_message
# If there is no username and password, please comment the following line:
mqttc.username_pw_set(mqtt_username, password=mqtt_password)
# Connect to mqtt broker, the heartbeat time is 60s
mqttc.connect(mqtt_ip, mqtt_port, 60)
mqttc.subscribe(mqtt_rx_topic, 0)
mqttc.loop_forever()