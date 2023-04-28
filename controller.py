"""
FPS MODE
------------------------------------------------------------------------------------------------
Left stick to control if drone laterally moves left, right, forwards or backwards
- moves laterally left/right : roll
- forwards : thrust
- backwards : loiter
Right stick to control if drone faces up down, left, or right. 
- faces up/down : pitch
- left/right : yaw
When survey mode is enabled, left Trigger to control wether pitch controls altitude or just camera mount. 
When pressed :
- Left bumper to descend
- Right bumper to ascend
Right trigger to take pictures or start/stop video. Camera mode in settings. 
"""

import pygame
from command_protocol import Protocol

class Mapping:

    def __init__(self):
        super().__init__()
        self.commands = Protocol()

    def handle_event(self, joystick_values):

        pitch = joystick_values["Right stick Y"]
        roll = joystick_values["Left stick X"]
        thrust = joystick_values["Left stick Y"] 
        yaw = joystick_values["Right stick X"]
        button_1 = 0
        button_2 = 0

        self.commands.manual_command(pitch, roll, thrust, yaw, button_1, button_2)

    def main(self):
        # Initialize Pygame and the joystick module
        pygame.init()
        pygame.joystick.init()

        # Get the number of connected joysticks
        joystick_count = pygame.joystick.get_count()

        # Create a dictionary to map axis and button indexes to their names
        axis_names = {
            0: "Left stick X", # -1 left to 1 right
            1: "Left stick Y", # -1 up to 1 down
            2: "Right stick X", # -1 left to 1 right
            3: "Right stick Y", # -1 up to 1 down
            4: "Left Trigger", # -1 to 1
            5: "Right trigger" # -1 to 1
        }

        button_names = {
            0: "A", # boolean
            1: "B", # boolean
            2: "X", # boolean
            3: "Y", # boolean
            4: "Left bumper", # boolean
            5: "Right bumper", # boolean
            6: "Select", # boolean
            7: "Start", # boolean
            8: "Left stick", # boolean
            9: "Right stick" # boolean
        }

        # Create an empty dictionary for the joystick values
        joystick_values = {}
        while True:
            for event in pygame.event.get():
                # Iterate through each joystick
                for i in range(joystick_count):
                    joystick = pygame.joystick.Joystick(i)
                    joystick.init()

                    # Get the number of axes and buttons for the current joystick
                    num_axes = joystick.get_numaxes()
                    num_buttons = joystick.get_numbuttons()

                    # Iterate through each axis of the current joystick and add it to the dictionary
                    for j in range(num_axes):
                        axis_value = joystick.get_axis(j)
                        axis_name = axis_names.get(j, f"Axis {j}")
                        joystick_values[axis_name] = axis_value

                    # Iterate through each button of the current joystick and add it to the dictionary
                    for k in range(num_buttons):
                        button_value = joystick.get_button(k)
                        button_name = button_names.get(k, f"Button {k}")
                        joystick_values[button_name] = button_value

                # Print the dictionary of joystick values
                joystick_values = {key: float("{:.3f}".format(value)) for key, value in joystick_values.items()}
                print("Joystick values:", joystick_values)
                self.handle_event(joystick_values)

if "__main__" == __name__:
    control = Mapping()
    control.main()