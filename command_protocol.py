from pymavlink import mavutil
from dronekit import VehicleMode
from command_messages import MavCmd

class Protocol():
    TARGET_SYSTEM = 1 # The system ID of the target MAVLink system. (1 for the autopilot) Component which should execute the command, 0 for all components
    TARGET_COMPONENT = 1 # The component ID of the target component on the target system. (mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1 for the autopilot), 0 for all components

    def __init__(self, port, baud):
        self.vehicle = mavutil.mavlink_connection(port, baud)
        
    def send_mavlink_command(self, command, params, frame):
        """
        MAV_CMD : Send a command with up to seven parameters to the MAV. 
        """
        if isinstance(params[5], float) and isinstance(params[6], float):
            # COMMAND_LONG is required for commands that mandate float values in params 5 and 6. The command microservice is documented at https://mavlink.io/en/services/command.html
            self.vehicle.mav.command_long_send(
                self.TARGET_SYSTEM, 
                self.TARGET_COMPONENT, 
                command, # Command ID (of command to send). ex : mavutil.mavlink.MAV_CMD_NAV_WAYPOINT 
                0,  # Confirmation - 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
                params 
            )
        else:
            # COMMAND_INT is generally preferred when sending MAV_CMD commands where param 5 and param 6 contain latitude/longitude data, as sending these in floats can result in a significant loss of precision. 
            self.vehicle.mav.command_int_send(
                self.TARGET_SYSTEM, 
                self.TARGET_COMPONENT,
                frame, # The coordinate system of the COMMAND. https://mavlink.io/en/messages/common.html#MAV_FRAME
                command, # Command ID (of command to send). ex : mavutil.mavlink.MAV_CMD_NAV_WAYPOINT 
                0, # current
                0, # autocontinue
                params 
            )

    def cancel_mavlink_command(self, command):
        """
        MAV_CMD - WIP !!! Cancel a long running command. 
        The target system should respond with a COMMAND_ACK to the original command with result=MAV_RESULT_CANCELLED if the long running process was cancelled. 
        If it has already completed, the cancel action can be ignored. The cancel action can be retried until some sort of acknowledgement to the original command has been received. 
        The command microservice is documented at https://mavlink.io/en/services/command.html
        """
        self.vehicle.mav.command_cancel_send(
            self.TARGET_SYSTEM, 
            self.TARGET_COMPONENT,  
            command, # Command ID (of command to cancel). ex : mavutil.mavlink.MAV_CMD_NAV_WAYPOINT 
        )

    def acknowledge_mavlink_command(self, command):
        """
        MAV_RESULT : Command acknowledgement. 
        Includes result (success, failure, still in progress) and may include progress information and additional detail about failure reasons.
        """
        ack_msg = self.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg.command == command:
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED: 
                command_status = str(command) + ' : Accepted'
            elif ack_msg.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                command_status = str(command) + ' : Temporarily rejected.'
            elif ack_msg.result == mavutil.mavlink.MAV_RESULT_DENIED:
                command_status = str(command) + ' : Denied'
            elif ack_msg.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
                command_status = str(command) + ' : Unsupported'
            elif ack_msg.result == mavutil.mavlink.MAV_RESULT_FAILED:
                command_status = str(command) + ' : Failed'
            elif ack_msg.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                command_status = str(command) + ' : In progress'
            elif ack_msg.result == mavutil.mavlink.	MAV_RESULT_CANCELLED:
                command_status = str(command) + ' : Cancelled'
            else:
                command_status = str(command) + ' : Failed'

            return command_status            

    def manual_command(self, pitch, roll, thrust, yaw, button_1, button_2):

        msg = self.vehicle.mav.manual_control_encode(
            self.TARGET_SYSTEM, 
            pitch, # X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
            roll, # Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
            thrust, # Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
            yaw, # R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
            button_1, # A bitfield corresponding to the joystick buttons' 0-15 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
            button_2, # A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 16.
        )

        self.mav.send(msg)

    def set_mode(self, mode):
        """
        Set mode to auto, guided or manual.
        """
        if mode == "MANUAL":
            try:
                custom_mode = 0
                base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

                # Send the command
                self.vehicle.mav.command_long_send(
                    self.TARGET_SYSTEM,  # target_system
                    mavutil.mavlink.MAV_COMP_ID_SYSTEM_CONTROL,  # target_component
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # command
                    0,  # confirmation
                    base_mode,  # param1
                    custom_mode,  # param2
                    0,  # param3
                    0,  # param4
                    0,  # param5
                    0,  # param6
                    0  # param7
                )
            except:
                self.vehicle.mode = VehicleMode('MANUAL')

        elif mode == "AUTO":
            try:
                # Set the mode to auto
                self.vehicle.mav.set_mode_send(
                    self.TARGET_SYSTEM, # target system
                    mavutil.mavlink.MAV_MODE_AUTO_GUIDED)  # base mode
            except:
                self.vehicle.mode = VehicleMode("AUTO")

        elif mode == "GUIDED":
            try:
                # Set the mode to auto
                self.vehicle.mav.command_long_send(
                    self.TARGET_SYSTEM,
                    self.TARGET_COMPONENT,
                    mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,  # command
                    0,                          # confirmation
                    1,                          # param 1, set to 1 to enable guided mode
                    0, 0, 0, 0, 0, 0)          # param 2 - 7 not used
            except:
                self.vehicle.mode = VehicleMode("GUIDED")