from pymavlink import mavutil


class CommandProtocol(object):

    def mav_cmd_nav_waypoint(self, hold, accept_radius, pass_radius, yaw, latitude, longitude, altitude):
        """
        Navigate to waypoint.
            hold: Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
            accept_radius: Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)
            pass_radius: 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
            yaw: Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        params = [hold, accept_radius, pass_radius, yaw, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_loiter_unlim(self, radius, yaw, latitude, longitude, altitude):
        """
        Loiter around this waypoint an unlimited amount of time
            radius: Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise
            yaw: Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
        params = [0, 0, radius, yaw, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_loiter_turns(self, turns, heading_required, radius, xtrack_location, latitude, longitude, altitude):
        """
        Loiter around this waypoint for X turns
            turns: Number of turns.
            heading_required: Leave loiter circle only once heading towards the next waypoint (0 = False)
            radius: Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise
            xtrack_location: Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS
        params = [turns, heading_required, radius, xtrack_location, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_loiter_time(self, time, heading_required, radius, xtrack_location, latitude, longitude, altitude):
        """
        Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint.
            time: Loiter time (only starts once Lat, Lon and Alt is reached).
            heading_required: Leave loiter circle only once heading towards the next waypoint (0 = False)
            radius: Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise.
            xtrack_location: Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME
        params = [time, heading_required, radius, xtrack_location, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_return_to_launch(self):
        """
        Return to launch location
        """
        command = mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_land(self, abort_alt, land_mode, yaw_angle, latitude, longitude, altitude):
        """
        Land at location.
            abort_alt: Minimum target altitude if landing is aborted (0 = undefined/use system default).
            land_mode: Precision land mode.
            yaw_angle: Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
            latitude: Latitude.
            longitude: Longitude.
            altitude: Landing altitude (ground level in current frame).
        """
        command = mavutil.mavlink.MAV_CMD_NAV_LAND
        params = [abort_alt, land_mode, 0, yaw_angle, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_takeoff(self, pitch, yaw, latitude, longitude, altitude):
        """
        Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.
            pitch: Minimum pitch (if airspeed sensor present), desired pitch without sensor
            yaw: Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        params = [pitch, 0, 0, yaw, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_land_local(self, target, offset, descend_rate, yaw, y_position, x_position, z_position):
        """
        Land at local position (local frame only)
            target: Landing target number (if available)
            offset: Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land
            descend_rate: Landing descend rate
            yaw: Desired yaw angle
            y_position: Y-axis position
            x_position: X-axis position
            z_position: Z-axis / ground level position
        """
        command = mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL
        params = [target, offset, descend_rate, yaw, y_position, x_position, z_position]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_takeoff_local(self, pitch, ascend_rate, yaw, y_position, x_position, z_position):
        """
        Takeoff from local position (local frame only)
            pitch: Minimum pitch (if airspeed sensor present), desired pitch without sensor
            ascend_rate: Takeoff ascend rate
            yaw: Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these
            y_position: Y-axis position
            x_position: X-axis position
            z_position: Z-axis position
        """
        command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL
        params = [pitch, 0, ascend_rate, yaw, y_position, x_position, z_position]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_follow(self, following, ground_speed, radius, yaw, latitude, longitude, altitude):
        """
        Vehicle following, i.e. this waypoint represents the position of a moving vehicle
            following: Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation
            ground_speed: Ground speed of vehicle to be followed
            radius: Radius around waypoint. If positive loiter clockwise, else counter-clockwise
            yaw: Desired yaw angle.
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_FOLLOW
        params = [following, ground_speed, radius, yaw, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_continue_and_change_alt(self, action, altitude):
        """
        Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
            action: Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.
            altitude: Desired altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT
        params = [action, 0, 0, 0, 0, 0, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_loiter_to_alt(self, heading_required, radius, xtrack_location, latitude, longitude, altitude):
        """
        Begin loiter at the specified Latitude and Longitude.  
        If Lat=Lon=0, then loiter at the current position.  
        Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. 
        Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint.
            heading_required: Leave loiter circle only once heading towards the next waypoint (0 = False)
            radius: Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.
            xtrack_location: Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TO_ALT
        params = [heading_required, radius, 0, xtrack_location, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_follow(self, system_id, altitude_mode, altitude, time_to_land):
        """
        Begin following a target
            system_id: System ID (of the FOLLOW_TARGET beacon). 
            Send 0 to disable follow-me and return to the default position hold mode.
            altitude_mode: Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.
            altitude: Altitude above home. (used if mode=2)
            time_to_land: Time to land in which the MAV should go to the default position hold mode after a message RX timeout.
        """
        command = mavutil.mavlink.MAV_CMD_DO_FOLLOW
        params = [system_id, 0, 0, altitude_mode, altitude, 0, time_to_land]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_follow_reposition(self, camera_q1, camera_q2, camera_q3, camera_q4, altitude_offset, x_offset, y_offset):
        """
        Reposition the MAV after a follow target command has been sent
            camera_q1: Camera q1 (where 0 is on the ray from the camera to the tracking device)
            camera_q2: Camera q2
            camera_q3: Camera q3
            camera_q4: Camera q4
            altitude_offset: altitude offset from target
            x_offset: X offset from target
            y_offset: Y offset from target
        """
        command = mavutil.mavlink.MAV_CMD_DO_FOLLOW_REPOSITION
        params = [camera_q1, camera_q2, camera_q3, camera_q4, altitude_offset, x_offset, y_offset]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_orbit(self, radius, velocity, yaw_behavior, orbits, latitude_x, longitude_y, altitude_z):
        """
        Start orbiting on the circumference of a circle defined by the parameters. Setting values to NaN/INT32_MAX (as appropriate) results in using defaults.
            radius: Radius of the circle. Positive: orbit clockwise. Negative: orbit counter-clockwise. NaN: Use vehicle default radius, or current radius if already orbiting.
            velocity: Tangential Velocity. NaN: Use vehicle default velocity, or current velocity if already orbiting.
            yaw_behavior: Yaw behavior of the vehicle.
            orbits: Orbit around the centre point for this many radians (i.e. for a three-quarter orbit set 270*Pi/180). 0: Orbit forever. NaN: Use vehicle default, or current value if already orbiting.
            latitude_x: Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. INT32_MAX (or NaN if sent in COMMAND_LONG): Use current vehicle position, or current center if already orbiting.
            longitude_y: Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. INT32_MAX (or NaN if sent in COMMAND_LONG): Use current vehicle position, or current center if already orbiting.
            altitude_z: Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle altitude.
        """
        command = mavutil.mavlink.MAV_CMD_DO_ORBIT
        params = [radius, velocity, yaw_behavior, orbits, latitude_x, longitude_y, altitude_z]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_roi(self, roi_mode, wp_index, roi_index, x, y, z):
        """
        Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
            roi_mode: Region of interest mode.
            wp_index: Waypoint index/ target ID. (see MAV_ROI enum)
            roi_index: ROI index (allows a vehicle to manage multiple ROI's)
            x: x the location of the fixed ROI (see MAV_FRAME)
            y: y
            z: z
        """
        command = mavutil.mavlink.MAV_CMD_NAV_ROI
        params = [roi_mode, wp_index, roi_index, 0, x, y, z]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_pathplanning(self, local_ctrl, global_ctrl, yaw, latitude_x, longitude_y, altitude_z):
        """
        Control autonomous path planning on the MAV.
            local_ctrl: 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
            global_ctrl: 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
            yaw: Yaw angle at goal
            latitude_x: Latitude/X of goal
            longitude_y: Longitude/Y of goal
            altitude_z: Altitude/Z of goal
        """
        command = mavutil.mavlink.MAV_CMD_NAV_PATHPLANNING
        params = [local_ctrl, global_ctrl, 0, yaw, latitude_x, longitude_y, altitude_z]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_spline_waypoint(self, hold, latitude_x, longitude_y, altitude_z):
        """
        Navigate to waypoint using a spline path.
            hold: Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)
            latitude_x: Latitude/X of goal
            longitude_y: Longitude/Y of goal
            altitude_z: Altitude/Z of goal
        """
        command = mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT
        params = [hold, 0, 0, 0, latitude_x, longitude_y, altitude_z]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_vtol_takeoff(self, transition_heading, yaw_angle, latitude, longitude, altitude):
        """
        Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.).
            transition_heading: Front transition heading.
            yaw_angle: Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF
        params = [0, transition_heading, 0, yaw_angle, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_vtol_land(self, land_options, approach_altitude, yaw, latitude, longitude, ground_altitude):
        """
        Land using VTOL mode
            land_options: Landing behaviour.
            approach_altitude: Approach altitude (with the same reference as the Altitude field). NaN if unspecified.
            yaw: Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).
            latitude: Latitude
            longitude: Longitude
            ground_altitude: Altitude (ground level) relative to the current coordinate frame. NaN to use system default landing altitude (ignore value).
        """
        command = mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND
        params = [land_options, 0, approach_altitude, yaw, latitude, longitude, ground_altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_guided_enable(self, enable):
        """
        hand control over to an external controller
            enable: On / Off (> 0.5f on)
        """
        command = mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE
        params = [enable, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_delay(self, delay, hour, minute, second):
        """
        Delay the next navigation command a number of seconds or until a specified time
            delay: Delay (-1 to enable time-of-day fields)
            hour: hour (24h format, UTC, -1 to ignore)
            minute: minute (24h format, UTC, -1 to ignore)
            second: second (24h format, UTC, -1 to ignore)
        """
        command = mavutil.mavlink.MAV_CMD_NAV_DELAY
        params = [delay, hour, minute, second, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_payload_place(self, max_descent, latitude, longitude, altitude):
        """
        Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload.
            max_descent: Maximum distance to descend.
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_PAYLOAD_PLACE
        params = [max_descent, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_last(self):
        """
        NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
        """
        command = mavutil.mavlink.MAV_CMD_NAV_LAST
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_condition_delay(self, delay):
        """
        Delay mission state machine.
            delay: Delay
        """
        command = mavutil.mavlink.MAV_CMD_CONDITION_DELAY
        params = [delay, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_condition_change_alt(self, rate, altitude):
        """
        Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached.
            rate: Descent / Ascend rate.
            altitude: Target Altitude
        """
        command = mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT
        params = [rate, 0, 0, 0, 0, 0, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_condition_distance(self, distance):
        """
        Delay mission state machine until within desired distance of next NAV point.
            distance: Distance.
        """
        command = mavutil.mavlink.MAV_CMD_CONDITION_DISTANCE
        params = [distance, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_condition_yaw(self, angle, angular_speed, direction, relative):
        """
        Reach a certain target angle.
            angle: target angle, 0 is north
            angular_speed: angular speed
            direction: direction: -1: counter clockwise, 1: clockwise
            relative: 0: absolute angle, 1: relative offset
        """
        command = mavutil.mavlink.MAV_CMD_CONDITION_YAW
        params = [angle, angular_speed, direction, relative, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_condition_last(self):
        """
        NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
        """
        command = mavutil.mavlink.MAV_CMD_CONDITION_LAST
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_mode(self, mode, custom_mode, custom_submode):
        """
        Set system mode.
            mode: Mode
            custom_mode: Custom mode - this is system specific, please refer to the individual autopilot specifications for details.
            custom_submode: Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_MODE
        params = [mode, custom_mode, custom_submode, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_jump(self, number, repeat):
        """
        Jump to the desired command in the mission list.  Repeat this action only the specified number of times
            number: Sequence number
            repeat: Repeat count
        """
        command = mavutil.mavlink.MAV_CMD_DO_JUMP
        params = [number, repeat, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_change_speed(self, speed_type, speed, throttle):
        """
        Change speed and/or throttle set points. The value persists until it is overridden or there is a mode change.
            speed_type: Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
            speed: Speed (-1 indicates no change, -2 indicates return to default vehicle speed)
            throttle: Throttle (-1 indicates no change, -2 indicates return to default vehicle throttle value)
        """
        command = mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED
        params = [speed_type, speed, throttle, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_home(self, use_current, yaw, latitude, longitude, altitude):
        """
                   Sets the home position to either to the current position or a specified position.           The home position is the default position that the system will return to and land on.           The position is set automatically by the system during the takeoff (and may also be set using this command).           Note: the current home position may be emitted in a HOME_POSITION message on request (using MAV_CMD_REQUEST_MESSAGE with param1=242).
            use_current: Use current (1=use current location, 0=use specified location)
            yaw: Yaw angle. NaN to use default heading
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_HOME
        params = [use_current, 0, 0, yaw, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_parameter(self, number, value):
        """
        Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
            number: Parameter number
            value: Parameter value
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER
        params = [number, value, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_relay(self, instance, setting):
        """
        Set a relay to a condition.
            instance: Relay instance number.
            setting: Setting. (1=on, 0=off, others possible depending on system hardware)
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_RELAY
        params = [instance, setting, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_repeat_relay(self, instance, count, time):
        """
        Cycle a relay on and off for a desired number of cycles with a desired period.
            instance: Relay instance number.
            count: Cycle count.
            time: Cycle time.
        """
        command = mavutil.mavlink.MAV_CMD_DO_REPEAT_RELAY
        params = [instance, count, time, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_servo(self, instance, pwm):
        """
        Set a servo to a desired PWM value.
            instance: Servo instance number.
            pwm: Pulse Width Modulation.
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_SERVO
        params = [instance, pwm, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_repeat_servo(self, instance, pwm, count, time):
        """
        Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
            instance: Servo instance number.
            pwm: Pulse Width Modulation.
            count: Cycle count.
            time: Cycle time.
        """
        command = mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO
        params = [instance, pwm, count, time, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_flighttermination(self, terminate):
        """
        Terminate flight immediately.           Flight termination immediately and irreversably terminates the current flight, returning the vehicle to ground.           The vehicle will ignore RC or other input until it has been power-cycled.           Termination may trigger safety measures, including: disabling motors and deployment of parachute on multicopters, and setting flight surfaces to initiate a landing pattern on fixed-wing).           On multicopters without a parachute it may trigger a crash landing.           Support for this command can be tested using the protocol bit: MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION.           Support for this command can also be tested by sending the command with param1=0 (< 0.5); the ACK should be either MAV_RESULT_FAILED or MAV_RESULT_UNSUPPORTED.
            terminate: Flight termination activated if > 0.5. Otherwise not activated and ACK with MAV_RESULT_FAILED.
        """
        command = mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION
        params = [terminate, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_change_altitude(self, altitude, frame):
        """
        Change altitude set point.
            altitude: Altitude.
            frame: Frame of new altitude.
        """
        command = mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE
        params = [altitude, frame, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_actuator(self, actuator_1, actuator_2, actuator_3, actuator_4, actuator_5, actuator_6, index):
        """
        Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).
            actuator_1: Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.
            actuator_2: Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.
            actuator_3: Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.
            actuator_4: Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.
            actuator_5: Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.
            actuator_6: Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.
            index: Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_ACTUATOR
        params = [actuator_1, actuator_2, actuator_3, actuator_4, actuator_5, actuator_6, index]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_land_start(self, latitude, longitude, altitude):
        """
        Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. 	  It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. 	  The Latitude/Longitude/Altitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_DO_LAND_START
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_rally_land(self, altitude, speed):
        """
        Mission command to perform a landing from a rally point.
            altitude: Break altitude
            speed: Landing speed
        """
        command = mavutil.mavlink.MAV_CMD_DO_RALLY_LAND
        params = [altitude, speed, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_go_around(self, altitude):
        """
        Mission command to safely abort an autonomous landing.
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_DO_GO_AROUND
        params = [altitude, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_reposition(self, speed, bitmask, radius, yaw, latitude, longitude, altitude):
        """
        Reposition the vehicle to a specific WGS84 global position.
            speed: Ground speed, less than 0 (-1) for default
            bitmask: Bitmask of option flags.
            radius: Loiter radius for planes. Positive values only, direction is controlled by Yaw value. A value of zero or NaN is ignored. 
            yaw: Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_DO_REPOSITION
        params = [speed, bitmask, radius, yaw, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_pause_continue(self, keep_going):
        """
        If in a GPS controlled position mode, hold the current position or continue.
            keep_going: 0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.
        """
        command = mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE
        params = [keep_going, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_reverse(self, reverse):
        """
        Set moving direction to forward or reverse.
            reverse: Direction (0=Forward, 1=Reverse)
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_REVERSE
        params = [reverse, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_roi_location(self, gimbal_device_id, latitude, longitude, altitude):
        """
        Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message.
            gimbal_device_id: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
            latitude: Latitude of ROI location
            longitude: Longitude of ROI location
            altitude: Altitude of ROI location
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION
        params = [gimbal_device_id, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_roi_wpnext_offset(self, gimbal_device_id, pitch_offset, roll_offset, yaw_offset):
        """
        Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message.
            gimbal_device_id: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
            pitch_offset: Pitch offset from next waypoint, positive pitching up
            roll_offset: Roll offset from next waypoint, positive rolling to the right
            yaw_offset: Yaw offset from next waypoint, positive yawing to the right
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET
        params = [gimbal_device_id, 0, 0, 0, pitch_offset, roll_offset, yaw_offset]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_roi_none(self, gimbal_device_id):
        """
        Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position.
            gimbal_device_id: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_ROI_NONE
        params = [gimbal_device_id, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_roi_sysid(self, system_id, gimbal_device_id):
        """
        Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message.
            system_id: System ID
            gimbal_device_id: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_ROI_SYSID
        params = [system_id, gimbal_device_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_control_video(self, id, transmission, interval, recording):
        """
        Control onboard camera system.
            id: Camera ID (-1 for all)
            transmission: Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
            interval: Transmission mode: 0: video stream, >0: single images every n seconds
            recording: Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
        """
        command = mavutil.mavlink.MAV_CMD_DO_CONTROL_VIDEO
        params = [id, transmission, interval, recording, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_roi(self, roi_mode, wp_index, roi_index):
        """
        Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
            roi_mode: Region of interest mode.
            wp_index: Waypoint index/ target ID (depends on param 1).
            roi_index: Region of interest index. (allows a vehicle to manage multiple ROI's)
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_ROI
        params = [roi_mode, wp_index, roi_index, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_digicam_configure(self, mode, shutter_speed, aperture, iso, exposure, command_identity, engine_cut_off):
        """
        Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
            mode: Modes: P, TV, AV, M, Etc.
            shutter_speed: Shutter speed: Divisor number for one second.
            aperture: Aperture: F stop number.
            iso: ISO number e.g. 80, 100, 200, Etc.
            exposure: Exposure type enumerator.
            command_identity: Command Identity.
            engine_cut_off: Main engine cut-off time before camera trigger. (0 means no cut-off)
        """
        command = mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONFIGURE
        params = [mode, shutter_speed, aperture, iso, exposure, command_identity, engine_cut_off]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_digicam_control(self, session_control, zoom_absolute, zoom_relative, focus, shoot_command, command_identity, shot_id):
        """
        Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
            session_control: Session control e.g. show/hide lens
            zoom_absolute: Zoom's absolute position
            zoom_relative: Zooming step value to offset zoom from the current position
            focus: Focus Locking, Unlocking or Re-locking
            shoot_command: Shooting Command
            command_identity: Command Identity
            shot_id: Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.
        """
        command = mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL
        params = [session_control, zoom_absolute, zoom_relative, focus, shoot_command, command_identity, shot_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_mount_configure(self, mode, stabilize_roll, stabilize_pitch, stabilize_yaw, roll_input_mode, pitch_input_mode, yaw_input_mode):
        """
        Mission command to configure a camera or antenna mount
            mode: Mount operation mode
            stabilize_roll: stabilize roll? (1 = yes, 0 = no)
            stabilize_pitch: stabilize pitch? (1 = yes, 0 = no)
            stabilize_yaw: stabilize yaw? (1 = yes, 0 = no)
            roll_input_mode: roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
            pitch_input_mode: pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
            yaw_input_mode: yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)
        """
        command = mavutil.mavlink.MAV_CMD_DO_MOUNT_CONFIGURE
        params = [mode, stabilize_roll, stabilize_pitch, stabilize_yaw, roll_input_mode, pitch_input_mode, yaw_input_mode]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_mount_control(self, pitch, roll, yaw, altitude, latitude, longitude, mode):
        """
        Mission command to control a camera or antenna mount
            pitch: pitch depending on mount mode (degrees or degrees/second depending on pitch input).
            roll: roll depending on mount mode (degrees or degrees/second depending on roll input).
            yaw: yaw depending on mount mode (degrees or degrees/second depending on yaw input).
            altitude: altitude depending on mount mode.
            latitude: latitude, set if appropriate mount mode.
            longitude: longitude, set if appropriate mount mode.
            mode: Mount mode.
        """
        command = mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL
        params = [pitch, roll, yaw, altitude, latitude, longitude, mode]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_cam_trigg_dist(self, distance, shutter, trigger):
        """
        Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
            distance: Camera trigger distance. 0 to stop triggering.
            shutter: Camera shutter integration time. -1 or 0 to ignore
            trigger: Trigger camera once immediately. (0 = no trigger, 1 = trigger)
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_DIST
        params = [distance, shutter, trigger, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_fence_enable(self, enable):
        """
        Mission command to enable the geofence
            enable: enable? (0=disable, 1=enable, 2=disable_floor_only)
        """
        command = mavutil.mavlink.MAV_CMD_DO_FENCE_ENABLE
        params = [enable, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_parachute(self, action):
        """
        Mission item/command to release a parachute or enable/disable auto release.
            action: Action
        """
        command = mavutil.mavlink.MAV_CMD_DO_PARACHUTE
        params = [action, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_motor_test(self, instance, throttle_type, throttle, timeout, motor_count, test_order):
        """
        Command to perform motor test.
            instance: Motor instance number (from 1 to max number of motors on the vehicle).
            throttle_type: Throttle type (whether the Throttle Value in param3 is a percentage, PWM value, etc.)
            throttle: Throttle value.
            timeout: Timeout between tests that are run in sequence.
            motor_count: Motor count. Number of motors to test in sequence: 0/1=one motor, 2= two motors, etc. The Timeout (param4) is used between tests.
            test_order: Motor test order.
        """
        command = mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST
        params = [instance, throttle_type, throttle, timeout, motor_count, test_order, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_inverted_flight(self, inverted):
        """
        Change to/from inverted flight.
            inverted: Inverted flight. (0=normal, 1=inverted)
        """
        command = mavutil.mavlink.MAV_CMD_DO_INVERTED_FLIGHT
        params = [inverted, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_gripper(self, instance, action):
        """
        Mission command to operate a gripper.
            instance: Gripper instance number.
            action: Gripper action to perform.
        """
        command = mavutil.mavlink.MAV_CMD_DO_GRIPPER
        params = [instance, action, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_autotune_enable(self, enable, axis):
        """
        Enable/disable autotune.
            enable: Enable (1: enable, 0:disable).
            axis: Specify which axis are autotuned. 0 indicates autopilot default settings.
        """
        command = mavutil.mavlink.MAV_CMD_DO_AUTOTUNE_ENABLE
        params = [enable, axis, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_set_yaw_speed(self, yaw, speed, angle):
        """
        Sets a desired vehicle turn angle and speed change.
            yaw: Yaw angle to adjust steering by.
            speed: Speed.
            angle: Final angle. (0=absolute, 1=relative)
        """
        command = mavutil.mavlink.MAV_CMD_NAV_SET_YAW_SPEED
        params = [yaw, speed, angle, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_cam_trigg_interval(self, trigger_cycle, shutter_integration):
        """
        Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.
            trigger_cycle: Camera trigger cycle time. -1 or 0 to ignore.
            shutter_integration: Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL
        params = [trigger_cycle, shutter_integration, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_mount_control_quat(self, q1, q2, q3, q4):
        """
        Mission command to control a camera or antenna mount, using a quaternion as reference.
            q1: quaternion param q1, w (1 in null-rotation)
            q2: quaternion param q2, x (0 in null-rotation)
            q3: quaternion param q3, y (0 in null-rotation)
            q4: quaternion param q4, z (0 in null-rotation)
        """
        command = mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL_QUAT
        params = [q1, q2, q3, q4, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_guided_master(self, system_id, component_id):
        """
        set id of master controller
            system_id: System ID
            component_id: Component ID
        """
        command = mavutil.mavlink.MAV_CMD_DO_GUIDED_MASTER
        params = [system_id, component_id, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_guided_limits(self, timeout, min_altitude, max_altitude, horiz_move_limit):
        """
        Set limits for external control
            timeout: Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.
            min_altitude: Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.
            max_altitude: Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.
            horiz_move_limit: Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.
        """
        command = mavutil.mavlink.MAV_CMD_DO_GUIDED_LIMITS
        params = [timeout, min_altitude, max_altitude, horiz_move_limit, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_engine_control(self, start_engine, cold_start, height_delay):
        """
        Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines
            start_engine: 0: Stop engine, 1:Start Engine
            cold_start: 0: Warm start, 1:Cold start. Controls use of choke where applicable
            height_delay: Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.
        """
        command = mavutil.mavlink.MAV_CMD_DO_ENGINE_CONTROL
        params = [start_engine, cold_start, height_delay, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_set_mission_current(self, number, reset_mission):
        """
                   Set the mission item with sequence number seq as the current item and emit MISSION_CURRENT (whether or not the mission number changed).           If a mission is currently being executed, the system will continue to this new mission item on the shortest path, skipping any intermediate mission items. 	  Note that mission jump repeat counters are not reset unless param2 is set (see MAV_CMD_DO_JUMP param2).            This command may trigger a mission state-machine change on some systems: for example from MISSION_STATE_NOT_STARTED or MISSION_STATE_PAUSED to MISSION_STATE_ACTIVE.           If the system is in mission mode, on those systems this command might therefore start, restart or resume the mission.           If the system is not in mission mode this command must not trigger a switch to mission mode.            The mission may be "reset" using param2.           Resetting sets jump counters to initial values (to reset counters without changing the current mission item set the param1 to `-1`).           Resetting also explicitly changes a mission state of MISSION_STATE_COMPLETE to MISSION_STATE_PAUSED or MISSION_STATE_ACTIVE, potentially allowing it to resume when it is (next) in a mission mode.  	  The command will ACK with MAV_RESULT_FAILED if the sequence number is out of range (including if there is no mission item).
            number: Mission sequence value to set. -1 for the current mission item (use to reset mission without changing current mission item).
            reset_mission: Resets mission. 1: true, 0: false. Resets jump counters to initial values and changes mission state "completed" to be "active" or "paused".
        """
        command = mavutil.mavlink.MAV_CMD_DO_SET_MISSION_CURRENT
        params = [number, reset_mission, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_last(self):
        """
        NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
        """
        command = mavutil.mavlink.MAV_CMD_DO_LAST
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_preflight_calibration(self, gyro_temperature, magnetometer, ground_pressure, remote_control, accelerometer, compmot_or_airspeed, esc_or_baro):
        """
        Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.
            gyro_temperature: 1: gyro calibration, 3: gyro temperature calibration
            magnetometer: 1: magnetometer calibration
            ground_pressure: 1: ground pressure calibration
            remote_control: 1: radio RC calibration, 2: RC trim calibration
            accelerometer: 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration
            compmot_or_airspeed: 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration
            esc_or_baro: 1: ESC calibration, 3: barometer temperature calibration
        """
        command = mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
        params = [gyro_temperature, magnetometer, ground_pressure, remote_control, accelerometer, compmot_or_airspeed, esc_or_baro]

        self.send_mavlink_command(command, params)

    def mav_cmd_preflight_set_sensor_offsets(self, sensor_type, x_offset, y_offset, z_offset, fourth_dimension, fifth_dimension, sixth_dimension):
        """
        Set sensor offsets. This command will be only accepted if in pre-flight mode.
            sensor_type: Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer
            x_offset: X axis offset (or generic dimension 1), in the sensor's raw units
            y_offset: Y axis offset (or generic dimension 2), in the sensor's raw units
            z_offset: Z axis offset (or generic dimension 3), in the sensor's raw units
            4th_dimension: Generic dimension 4, in the sensor's raw units
            5th_dimension: Generic dimension 5, in the sensor's raw units
            6th_dimension: Generic dimension 6, in the sensor's raw units
        """
        command = mavutil.mavlink.MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS
        params = [sensor_type, x_offset, y_offset, z_offset, fourth_dimension, fifth_dimension, sixth_dimension]

        self.send_mavlink_command(command, params)

    def mav_cmd_preflight_uavcan(self, actuator_id):
        """
        Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named).
            actuator_id: 1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.
        """
        command = mavutil.mavlink.MAV_CMD_PREFLIGHT_UAVCAN
        params = [actuator_id, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_preflight_storage(self, parameter_storage, mission_storage, logging_rate):
        """
        Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
            parameter_storage: Action to perform on the persistent parameter storage
            mission_storage: Action to perform on the persistent mission storage
            logging_rate: Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: logging rate (e.g. set to 1000 for 1000 Hz logging)
        """
        command = mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE
        params = [parameter_storage, mission_storage, logging_rate, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_preflight_reboot_shutdown(self, autopilot, companion, component_action, component_id):
        """
        Request the reboot or shutdown of system components.
            autopilot: 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.
            companion: 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.
            component_action: 0: Do nothing for component, 1: Reboot component, 2: Shutdown component, 3: Reboot component and keep it in the bootloader until upgraded
            component_id: MAVLink Component ID targeted in param3 (0 for all components).
        """
        command = mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
        params = [autopilot, companion, component_action, component_id, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_override_goto(self, keep_going, position, frame, yaw, latitude_x, longitude_y, altitude_z):
        """
        Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position.
            keep_going: MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.
            position: MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.
            frame: Coordinate frame of hold point.
            yaw: Desired yaw angle.
            latitude_x: Latitude/X position.
            longitude_y: Longitude/Y position.
            altitude_z: Altitude/Z position.
        """
        command = mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO
        params = [keep_going, position, frame, yaw, latitude_x, longitude_y, altitude_z]

        self.send_mavlink_command(command, params)

    def mav_cmd_oblique_survey(self, distance, shutter, min_interval, positions, roll_angle, pitch_angle):
        """
        Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera.
            distance: Camera trigger distance. 0 to stop triggering.
            shutter: Camera shutter integration time. 0 to ignore
            min_interval: The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore.
            positions: Total number of roll positions at which the camera will capture photos (images captures spread evenly across the limits defined by param5).
            roll_angle: Angle limits that the camera can be rolled to left and right of center.
            pitch_angle: Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis.
        """
        command = mavutil.mavlink.MAV_CMD_OBLIQUE_SURVEY
        params = [distance, shutter, min_interval, positions, roll_angle, pitch_angle, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_mission_start(self, first_item, last_item):
        """
        start running a mission
            first_item: first_item: the first mission item to run
            last_item: last_item:  the last mission item to run (after this item is run, the mission ends)
        """
        command = mavutil.mavlink.MAV_CMD_MISSION_START
        params = [first_item, last_item]

        self.send_mavlink_command(command, params)

    def mav_cmd_actuator_test(self, value, timeout, output_function):
        """
        Actuator testing command. This is similar to MAV_CMD_DO_MOTOR_TEST but operates on the level of output functions, i.e. it is possible to test Motor1 independent from which output it is configured on. Autopilots typically refuse this command while armed.
            value: Output value: 1 means maximum positive output, 0 to center servos or minimum motor thrust (expected to spin), -1 for maximum negative (if not supported by the motors, i.e. motor is not reversible, smaller than 0 maps to NaN). And NaN maps to disarmed (stop the motors).
            timeout: Timeout after which the test command expires and the output is restored to the previous value. A timeout has to be set for safety reasons. A timeout of 0 means to restore the previous value immediately.
            output_function: Actuator Output function
        """
        command = mavutil.mavlink.MAV_CMD_ACTUATOR_TEST
        params = [value, timeout, 0, 0, output_function, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_configure_actuator(self, configuration, output_function):
        """
        Actuator configuration command.
            configuration: Actuator configuration action
            output_function: Actuator Output function
        """
        command = mavutil.mavlink.MAV_CMD_CONFIGURE_ACTUATOR
        params = [configuration, 0, 0, 0, output_function, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_component_arm_disarm(self, arm, force):
        """
        Arms / Disarms a component
            arm: 0: disarm, 1: arm
            force: 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
        """
        command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        params = [arm, force]

        self.send_mavlink_command(command, params)

    def mav_cmd_run_prearm_checks(self):
        """
        Instructs a target system to run pre-arm checks.           This allows preflight checks to be run on demand, which may be useful on systems that normally run them at low rate, or which do not trigger checks when the armable state might have changed.           This command should return MAV_RESULT_ACCEPTED if it will run the checks.           The results of the checks are usually then reported in SYS_STATUS messages (this is system-specific).           The command should return MAV_RESULT_TEMPORARILY_REJECTED if the system is already armed.
        """
        command = mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS
        params = []

        self.send_mavlink_command(command, params)

    def mav_cmd_illuminator_on_off(self, enable):
        """
        Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light).
            enable: 0: Illuminators OFF, 1: Illuminators ON
        """
        command = mavutil.mavlink.MAV_CMD_ILLUMINATOR_ON_OFF
        params = [enable]

        self.send_mavlink_command(command, params)

    def mav_cmd_get_home_position(self):
        """
        Request the home position from the vehicle. 	  The vehicle will ACK the command and then emit the HOME_POSITION message.
        """
        command = mavutil.mavlink.MAV_CMD_GET_HOME_POSITION
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_inject_failure(self, failure_unit, failure_type, instance):
        """
        Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting.
            failure_unit: The unit which is affected by the failure.
            failure_type: The type how the failure manifests itself.
            instance: Instance affected by failure (0 to signal all).
        """
        command = mavutil.mavlink.MAV_CMD_INJECT_FAILURE
        params = [failure_unit, failure_type, instance]

        self.send_mavlink_command(command, params)

    def mav_cmd_start_rx_pair(self, spektrum, rc_type):
        """
        Starts receiver pairing.
            spektrum: 0:Spektrum.
            rc_type: RC type.
        """
        command = mavutil.mavlink.MAV_CMD_START_RX_PAIR
        params = [spektrum, rc_type]

        self.send_mavlink_command(command, params)

    def mav_cmd_get_message_interval(self, message_id):
        """
                   Request the interval between messages for a particular MAVLink message ID.           The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message.
            message_id: The MAVLink message ID
        """
        command = mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL
        params = [message_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_set_message_interval(self, message_id, interval, response_target):
        """
        Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.
            message_id: The MAVLink message ID
            interval: The interval between two messages. -1: disable. 0: request default rate (which may be zero).
            response_target: Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
        """
        command = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        params = [message_id, interval, response_target]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_message(self, message_id, req_param_1, req_param_2, req_param_3, req_param_4, req_param_5, response_target):
        """
        Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL).
            message_id: The MAVLink message ID of the requested message.
            req_param_1: Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).
            req_param_2: The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
            req_param_3: The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
            req_param_4: The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
            req_param_5: The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).
            response_target: Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE
        params = [message_id, req_param_1, req_param_2, req_param_3, req_param_4, req_param_5, response_target]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_protocol_version(self, protocol):
        """
        Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message
            protocol: 1: Request supported protocol versions by all nodes on the network
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_PROTOCOL_VERSION
        params = [protocol, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_autopilot_capabilities(self, version):
        """
        Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message
            version: 1: Request autopilot version
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
        params = [version, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_camera_information(self, capabilities):
        """
        Request camera information (CAMERA_INFORMATION).
            capabilities: 0: No action 1: Request camera capabilities
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_INFORMATION
        params = [capabilities, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_camera_settings(self, settings):
        """
        Request camera settings (CAMERA_SETTINGS).
            settings: 0: No Action 1: Request camera settings
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_SETTINGS
        params = [settings, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_storage_information(self, storage_id, information):
        """
        Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.
            storage_id: Storage ID (0 for all, 1 for first, 2 for second, etc.)
            information: 0: No Action 1: Request storage information
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_STORAGE_INFORMATION
        params = [storage_id, information, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_storage_format(self, storage_id, format, reset_image_log):
        """
        Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.
            storage_id: Storage ID (1 for first, 2 for second, etc.)
            format: Format storage (and reset image log). 0: No action 1: Format storage
            reset_image_log: Reset Image Log (without formatting storage medium). This will reset CAMERA_CAPTURE_STATUS.image_count and CAMERA_IMAGE_CAPTURED.image_index. 0: No action 1: Reset Image Log
        """
        command = mavutil.mavlink.MAV_CMD_STORAGE_FORMAT
        params = [storage_id, format, reset_image_log, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_camera_capture_status(self, capture_status):
        """
        Request camera capture status (CAMERA_CAPTURE_STATUS)
            capture_status: 0: No Action 1: Request camera capture status
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS
        params = [capture_status, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_flight_information(self, flight_information):
        """
        Request flight information (FLIGHT_INFORMATION)
            flight_information: 1: Request flight information
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_FLIGHT_INFORMATION
        params = [flight_information, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_reset_camera_settings(self, reset):
        """
        Reset all camera settings to Factory Default
            reset: 0: No Action 1: Reset all settings
        """
        command = mavutil.mavlink.MAV_CMD_RESET_CAMERA_SETTINGS
        params = [reset, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_set_camera_mode(self, camera_mode):
        """
        Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming.
            camera_mode: Camera mode
        """
        command = mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE
        params = [0, camera_mode, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_set_camera_zoom(self, zoom_type, zoom_value):
        """
        Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success).
            zoom_type: Zoom type
            zoom_value: Zoom value. The range of valid values depend on the zoom type.
        """
        command = mavutil.mavlink.MAV_CMD_SET_CAMERA_ZOOM
        params = [zoom_type, zoom_value, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_set_camera_focus(self, focus_type, focus_value):
        """
        Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success).
            focus_type: Focus type
            focus_value: Focus value
        """
        command = mavutil.mavlink.MAV_CMD_SET_CAMERA_FOCUS
        params = [focus_type, focus_value, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_set_storage_usage(self, storage_id, usage):
        """
        Set that a particular storage is the preferred location for saving photos, videos, and/or other media (e.g. to set that an SD card is used for storing videos).           There can only be one preferred save location for each particular media type: setting a media usage flag will clear/reset that same flag if set on any other storage.           If no flag is set the system should use its default storage.           A target system can choose to always use default storage, in which case it should ACK the command with MAV_RESULT_UNSUPPORTED.           A target system can choose to not allow a particular storage to be set as preferred storage, in which case it should ACK the command with MAV_RESULT_DENIED.
            storage_id: Storage ID (1 for first, 2 for second, etc.)
            usage: Usage flags
        """
        command = mavutil.mavlink.MAV_CMD_SET_STORAGE_USAGE
        params = [storage_id, usage]

        self.send_mavlink_command(command, params)

    def mav_cmd_jump_tag(self, tag):
        """
        Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.
            tag: Tag.
        """
        command = mavutil.mavlink.MAV_CMD_JUMP_TAG
        params = [tag]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_jump_tag(self, tag, repeat):
        """
        Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number.
            tag: Target tag to jump to.
            repeat: Repeat count.
        """
        command = mavutil.mavlink.MAV_CMD_DO_JUMP_TAG
        params = [tag, repeat]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_gimbal_manager_pitchyaw(self, pitch_angle, yaw_angle, pitch_rate, yaw_rate, gimbal_manager_flags, gimbal_device_id):
        """
        High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager.
            pitch_angle: Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).
            yaw_angle: Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).
            pitch_rate: Pitch rate (positive to pitch up).
            yaw_rate: Yaw rate (positive to yaw to the right).
            gimbal_manager_flags: Gimbal manager flags to use.
            gimbal_device_id: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
        """
        command = mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
        params = [pitch_angle, yaw_angle, pitch_rate, yaw_rate, gimbal_manager_flags, gimbal_device_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_gimbal_manager_configure(self, sysid_primary_control, compid_primary_control, sysid_secondary_control, compid_secondary_control, gimbal_device_id):
        """
        Gimbal configuration to set which sysid/compid is in primary and secondary control.
            sysid_primary_control: Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
            compid_primary_control: Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
            sysid_secondary_control: Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
            compid_secondary_control: Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).
            gimbal_device_id: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
        """
        command = mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
        params = [sysid_primary_control, compid_primary_control, sysid_secondary_control, compid_secondary_control, gimbal_device_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_image_start_capture(self, interval, total_images, sequence_number):
        """
        Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values.
            interval: Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).
            total_images: Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.
            sequence_number: Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise set to 0. Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted.
        """
        command = mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE
        params = [0, interval, total_images, sequence_number, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_image_stop_capture(self):
        """
        Stop image capture sequence Use NaN for reserved values.
        """
        command = mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE
        params = [0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_camera_image_capture(self, number):
        """
        Re-request a CAMERA_IMAGE_CAPTURED message.
            number: Sequence number for missing CAMERA_IMAGE_CAPTURED message
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE
        params = [number, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_trigger_control(self, enable, reset, pause):
        """
        Enable or disable on-board camera triggering system.
            enable: Trigger enable/disable (0 for disable, 1 for start), -1 to ignore
            reset: 1 to reset the trigger sequence, -1 or 0 to ignore
            pause: 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore
        """
        command = mavutil.mavlink.MAV_CMD_DO_TRIGGER_CONTROL
        params = [enable, reset, pause]

        self.send_mavlink_command(command, params)

    def mav_cmd_camera_track_point(self, point_x, point_y, radius):
        """
        If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking.
            point_x: Point to track x value (normalized 0..1, 0 is left, 1 is right).
            point_y: Point to track y value (normalized 0..1, 0 is top, 1 is bottom).
            radius: Point radius (normalized 0..1, 0 is image left, 1 is image right).
        """
        command = mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT
        params = [point_x, point_y, radius]

        self.send_mavlink_command(command, params)

    def mav_cmd_camera_track_rectangle(self, top_left_corner_x, top_left_corner_y, bottom_right_corner_x, bottom_right_corner_y):
        """
        If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking.
            top_left_corner_x: Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
            top_left_corner_y: Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
            bottom_right_corner_x: Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
            bottom_right_corner_y: Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
        """
        command = mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE
        params = [top_left_corner_x, top_left_corner_y, bottom_right_corner_x, bottom_right_corner_y]

        self.send_mavlink_command(command, params)

    def mav_cmd_camera_stop_tracking(self):
        """
        Stops ongoing tracking.
        """
        command = mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING
        params = []

        self.send_mavlink_command(command, params)

    def mav_cmd_video_start_capture(self, stream_id, status_frequency):
        """
        Starts video capture (recording).
            stream_id: Video Stream ID (0 for all streams)
            status_frequency: Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency)
        """
        command = mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE
        params = [stream_id, status_frequency, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_video_stop_capture(self, stream_id):
        """
        Stop the current video capture (recording).
            stream_id: Video Stream ID (0 for all streams)
        """
        command = mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE
        params = [stream_id, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_video_start_streaming(self, stream_id):
        """
        Start video streaming
            stream_id: Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
        """
        command = mavutil.mavlink.MAV_CMD_VIDEO_START_STREAMING
        params = [stream_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_video_stop_streaming(self, stream_id):
        """
        Stop the given video stream
            stream_id: Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
        """
        command = mavutil.mavlink.MAV_CMD_VIDEO_STOP_STREAMING
        params = [stream_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_video_stream_information(self, stream_id):
        """
        Request video stream information (VIDEO_STREAM_INFORMATION)
            stream_id: Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION
        params = [stream_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_request_video_stream_status(self, stream_id):
        """
        Request video stream status (VIDEO_STREAM_STATUS)
            stream_id: Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)
        """
        command = mavutil.mavlink.MAV_CMD_REQUEST_VIDEO_STREAM_STATUS
        params = [stream_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_logging_start(self, format):
        """
        Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
            format: Format: 0: ULog
        """
        command = mavutil.mavlink.MAV_CMD_LOGGING_START
        params = [format, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_logging_stop(self):
        """
        Request to stop streaming log data over MAVLink
        """
        command = mavutil.mavlink.MAV_CMD_LOGGING_STOP
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_airframe_configuration(self, landing_gear_id, landing_gear_position):
        """
        
            landing_gear_id: Landing gear ID (default: 0, -1 for all)
            landing_gear_position: Landing gear position (Down: 0, Up: 1, NaN for no change)
        """
        command = mavutil.mavlink.MAV_CMD_AIRFRAME_CONFIGURATION
        params = [landing_gear_id, landing_gear_position, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_control_high_latency(self, enable):
        """
        Request to start/stop transmitting over the high latency telemetry
            enable: Control transmission over high latency telemetry (0: stop, 1: start)
        """
        command = mavutil.mavlink.MAV_CMD_CONTROL_HIGH_LATENCY
        params = [enable, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_panorama_create(self, horizontal_angle, vertical_angle, horizontal_speed, vertical_speed):
        """
        Create a panorama at the current position
            horizontal_angle: Viewing angle horizontal of the panorama (+- 0.5 the total angle)
            vertical_angle: Viewing angle vertical of panorama.
            horizontal_speed: Speed of the horizontal rotation.
            vertical_speed: Speed of the vertical rotation.
        """
        command = mavutil.mavlink.MAV_CMD_PANORAMA_CREATE
        params = [horizontal_angle, vertical_angle, horizontal_speed, vertical_speed]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_vtol_transition(self, state, immediate):
        """
        Request VTOL transition
            state: The target VTOL state. For normal transitions, only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.
            immediate: Force immediate transition to the specified MAV_VTOL_STATE. 1: Force immediate, 0: normal transition. Can be used, for example, to trigger an emergency "Quadchute". Caution: Can be dangerous/damage vehicle, depending on autopilot implementation of this command.
        """
        command = mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION
        params = [state, immediate]

        self.send_mavlink_command(command, params)

    def mav_cmd_arm_authorization_request(self, system_id):
        """
        Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. 		If approved the COMMAND_ACK message progress field should be set with period of time that this authorization is valid in seconds. 		If the authorization is denied COMMAND_ACK.result_param2 should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
            system_id: Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle
        """
        command = mavutil.mavlink.MAV_CMD_ARM_AUTHORIZATION_REQUEST
        params = [system_id]

        self.send_mavlink_command(command, params)

    def mav_cmd_set_guided_submode_standard(self):
        """
        This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
        """
        command = mavutil.mavlink.MAV_CMD_SET_GUIDED_SUBMODE_STANDARD
        params = []

        self.send_mavlink_command(command, params)

    def mav_cmd_set_guided_submode_circle(self, radius, latitude, longitude):
        """
        This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
            radius: Radius of desired circle in CIRCLE_MODE
            latitude: Target latitude of center of circle in CIRCLE_MODE
            longitude: Target longitude of center of circle in CIRCLE_MODE
        """
        command = mavutil.mavlink.MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE
        params = [radius, 0, 0, 0, latitude, longitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_condition_gate(self, geometry, usealtitude, latitude, longitude, altitude):
        """
        Delay mission state machine until gate has been reached.
            geometry: Geometry: 0: orthogonal to path between previous and next waypoint.
            usealtitude: Altitude: 0: ignore altitude
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_CONDITION_GATE
        params = [geometry, usealtitude, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_fence_return_point(self, latitude, longitude, altitude):
        """
        Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead.
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_fence_polygon_vertex_inclusion(self, vertex_count, inclusion_group, latitude, longitude):
        """
        Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
            vertex_count: Polygon vertex count
            inclusion_group: Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group, must be the same for all points in each polygon
            latitude: Latitude
            longitude: Longitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        params = [vertex_count, inclusion_group, 0, 0, latitude, longitude, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_fence_polygon_vertex_exclusion(self, vertex_count, latitude, longitude):
        """
        Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
            vertex_count: Polygon vertex count
            latitude: Latitude
            longitude: Longitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
        params = [vertex_count, 0, 0, 0, latitude, longitude, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_fence_circle_inclusion(self, radius, inclusion_group, latitude, longitude):
        """
        Circular fence area. The vehicle must stay inside this area.
            radius: Radius.
            inclusion_group: Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group
            latitude: Latitude
            longitude: Longitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
        params = [radius, inclusion_group, 0, 0, latitude, longitude, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_fence_circle_exclusion(self, radius, latitude, longitude):
        """
        Circular fence area. The vehicle must stay outside this area.
            radius: Radius.
            latitude: Latitude
            longitude: Longitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
        params = [radius, 0, 0, 0, latitude, longitude, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_nav_rally_point(self, latitude, longitude, altitude):
        """
        Rally point. You can have multiple rally points defined.
            latitude: Latitude
            longitude: Longitude
            altitude: Altitude
        """
        command = mavutil.mavlink.MAV_CMD_NAV_RALLY_POINT
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_uavcan_get_node_info(self):
        """
        Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.
        """
        command = mavutil.mavlink.MAV_CMD_UAVCAN_GET_NODE_INFO
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_adsb_out_ident(self):
        """
        Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air Traffic Controller in controlled airspace. This starts the IDENT which is then typically held for 18 seconds by the hardware per the Mode A, C, and S transponder spec.
        """
        command = mavutil.mavlink.MAV_CMD_DO_ADSB_OUT_IDENT
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_payload_prepare_deploy(self, operation_mode, approach_vector, ground_speed, altitude_clearance, latitude, longitude, altitude):
        """
        Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
            operation_mode: Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.
            approach_vector: Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will.
            ground_speed: Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.
            altitude_clearance: Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.
            latitude: Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)
            longitude: Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_PAYLOAD_PREPARE_DEPLOY
        params = [operation_mode, approach_vector, ground_speed, altitude_clearance, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_payload_control_deploy(self, operation_mode):
        """
        Control the payload deployment.
            operation_mode: Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.
        """
        command = mavutil.mavlink.MAV_CMD_PAYLOAD_CONTROL_DEPLOY
        params = [operation_mode, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_fixed_mag_cal_yaw(self, yaw, compassmask, latitude, longitude):
        """
        Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location.
            yaw: Yaw of vehicle in earth frame.
            compassmask: CompassMask, 0 for all.
            latitude: Latitude.
            longitude: Longitude.
        """
        command = mavutil.mavlink.MAV_CMD_FIXED_MAG_CAL_YAW
        params = [yaw, compassmask, latitude, longitude, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_do_winch(self, instance, action, length, rate):
        """
        Command to operate winch.
            instance: Winch instance number.
            action: Action to perform.
            length: Length of line to release (negative to wind).
            rate: Release rate (negative to wind).
        """
        command = mavutil.mavlink.MAV_CMD_DO_WINCH
        params = [instance, action, length, rate, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_waypoint_user_1(self, latitude, longitude, altitude):
        """
        User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_WAYPOINT_USER_1
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_waypoint_user_2(self, latitude, longitude, altitude):
        """
        User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_WAYPOINT_USER_2
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_waypoint_user_3(self, latitude, longitude, altitude):
        """
        User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_WAYPOINT_USER_3
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_waypoint_user_4(self, latitude, longitude, altitude):
        """
        User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_WAYPOINT_USER_4
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_waypoint_user_5(self, latitude, longitude, altitude):
        """
        User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_WAYPOINT_USER_5
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_spatial_user_1(self, latitude, longitude, altitude):
        """
        User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_SPATIAL_USER_1
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_spatial_user_2(self, latitude, longitude, altitude):
        """
        User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_SPATIAL_USER_2
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_spatial_user_3(self, latitude, longitude, altitude):
        """
        User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_SPATIAL_USER_3
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_spatial_user_4(self, latitude, longitude, altitude):
        """
        User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_SPATIAL_USER_4
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_spatial_user_5(self, latitude, longitude, altitude):
        """
        User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
            latitude: Latitude unscaled
            longitude: Longitude unscaled
            altitude: Altitude (MSL)
        """
        command = mavutil.mavlink.MAV_CMD_SPATIAL_USER_5
        params = [0, 0, 0, 0, latitude, longitude, altitude]

        self.send_mavlink_command(command, params)

    def mav_cmd_user_1(self):
        """
        User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        """
        command = mavutil.mavlink.MAV_CMD_USER_1
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_user_2(self):
        """
        User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        """
        command = mavutil.mavlink.MAV_CMD_USER_2
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_user_3(self):
        """
        User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        """
        command = mavutil.mavlink.MAV_CMD_USER_3
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_user_4(self):
        """
        User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        """
        command = mavutil.mavlink.MAV_CMD_USER_4
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_user_5(self):
        """
        User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
        """
        command = mavutil.mavlink.MAV_CMD_USER_5
        params = [0, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

    def mav_cmd_can_forward(self, bus):
        """
        Request forwarding of CAN packets from the given CAN bus to this component. CAN Frames are sent using CAN_FRAME and CANFD_FRAME messages
            bus: Bus number (0 to disable forwarding, 1 for first bus, 2 for 2nd bus, 3 for 3rd bus).
        """
        command = mavutil.mavlink.MAV_CMD_CAN_FORWARD
        params = [bus, 0, 0, 0, 0, 0, 0]

        self.send_mavlink_command(command, params)

