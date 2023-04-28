import urllib.request
import xml.etree.ElementTree as ET
import os
import re

@staticmethod
def common_cmd_to_methods():
    """
    ALL MAV_CMD ENUMS IN COMMON.XML TO METHODS
    """
    # download the xml file from the url
    url = 'https://raw.githubusercontent.com/mavlink/mavlink/master/message_definitions/v1.0/common.xml'
    response = urllib.request.urlopen(url)
    xml_str = response.read()

    # parse the xml string
    root = ET.fromstring(xml_str)

    # find the MAV_CMD enum
    enum = root.find(".//enum[@name='MAV_CMD']")

    all_methods = "from pymavlink import mavutil\n\n\n"
    all_methods += "class mav_cmd(object):\n\n"

    # iterate over the enum entries
    for entry in enum.findall("entry"):

        params = []
        arguments = ""
        entry_name = entry.get('name').lower()
        entry_desc = entry.findtext('description').replace('\n', ' ').rstrip()

        # iterate over the entry's params
        for param in entry.findall("param"):
            param_label = param.get('label')
            param_index = param.get('index')
            
            # print the item of the list and its description as a comment
            if param_label:
                item_name = param_label.lower().replace(' ', '_').replace('continue', 'keep_going').rstrip()
                item_name = re.sub(r'^(\d+)(\w+)', lambda x: str(int(x.group(1))) + x.group(2), item_name)
                item_name = re.sub('[^A-Za-z0-9]+', '_', item_name)
                

                if int(param_index) == 0:
                    entry_desc += "\n\n        Args:"

                entry_desc += "\n            " + item_name + ": " + param.text 

                if int(param_index) in range(1,8): 
                    arguments += ", "

                arguments += item_name 
                params.append(item_name)
            else:
                params.append("0")

        parameters = "params = [" + ", ".join(params) + "]\n"
        command = "command = mavutil.mavlink." + entry.get('name')
        method = "    def " + entry_name + "(self" + arguments + "):\n"
        method += '        """\n        ' + entry_desc + "\n" + '        """' + "\n" 
        method += "        " + command + "\n" 
        method += "        " + parameters + "\n"
        method += "        self.send_mavlink_command(command, params)\n\n"

        all_methods += method
        
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print(dir_path)
    with open(dir_path + '/common.py', 'w') as f:
        f.write(all_methods)


if __name__ == "__main__":
    common_cmd_to_methods()