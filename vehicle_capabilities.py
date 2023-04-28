import json
import os
import time
import requests
import xml.etree.ElementTree as ET
import pymavlink.dialects.v20 as mavlink
from pymavlink import mavutil


@staticmethod
def get_common_enums_commands_messages(): 
    # Fetch the MAVLink common message definitions XML file from GitHub
    url = 'https://raw.githubusercontent.com/mavlink/mavlink/master/message_definitions/v1.0/common.xml'
    response = requests.get(url)

    # Parse the XML file and extract the entry names
    root = ET.fromstring(response.content)
    messages = [entry.attrib['name'] for entry in root.iter('message')]
    enum = root.find(".//enum[@name='MAV_CMD']")
    commands = [entry.attrib['name'] for entry in enum.findall("entry")]
    enums = [enum.attrib['name'] for enum in root.iter('enum')]

    return enums, commands, messages

def available_enums_commands_messages(self, enums, commands, messages):

    while True:
        msg = self.vehicle.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            break
        time.sleep(0.1)

    supported_enums = [enum for enum in enums if enum in mavlink.MAVLINK_MESSAGE_INFO]
    supported_commands = [command for command in commands if command in mavlink.MAV_CMD]
    supported_messages = [message for message in messages if message in mavlink.MAVLINK_MESSAGE_INFO]

    return supported_enums, supported_commands, supported_messages

@staticmethod
def export_capabilites_to_json(supported_enums, supported_commands, supported_messages):
    
    # Create a sample dictionary to write to the JSON file
    vehicle_capabilites = {"enums": supported_enums, "commands": supported_commands, "messages": supported_messages}

    # Get the path to the active folder where the Python script is located
    folder_path = os.path.dirname(os.path.abspath(__file__))

    # Construct the file path for the JSON file in the active folder
    file_path = os.path.join(folder_path, "\vehicle_capabilities.json")

    # Write the dictionary to the JSON file
    with open(file_path, "w") as f:
        json.dump(vehicle_capabilites, f)