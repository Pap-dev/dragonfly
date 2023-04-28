import xml.etree.ElementTree as ET
import urllib.request
import json

@staticmethod
def mav_cmd_param_labels_to_json():
    """
    This function collects every parameters and their description from MAV_CMD enums in common.xml and saves them as a JSON file.
    """
    # download the xml file from the url
    url = 'https://raw.githubusercontent.com/mavlink/mavlink/master/message_definitions/v1.0/common.xml'
    response = urllib.request.urlopen(url)
    xml_str = response.read()

    # parse the xml string
    root = ET.fromstring(xml_str)

    # Extract param labels and their descriptions from the MAV_CMD enum
    param_labels_dict = {}
    for enum in root.iter('enum'):
        if enum.get('name') == 'MAV_CMD':
            for entry in enum.iter('entry'):
                for param in entry.iter('param'):
                    label = param.get('label')
                    description = param.text.strip() if param.text else ""
                    if label:
                        param_labels_dict[label] = description

    # Save the dictionary as a JSON file
    with open('mav_cmd_param_labels.json', 'w') as json_file:
        json.dump(param_labels_dict, json_file, indent=4)

    print("mav_cmd_param_labels.json has been saved in the current folder.")

    
if __name__ == "__main__":
    mav_cmd_param_labels_to_json()