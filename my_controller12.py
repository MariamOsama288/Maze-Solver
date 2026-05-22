"""maze solver controller"""

from controller import Robot, DistanceSensor, Motor, Supervisor
import numpy as np
import math

#-------------------------------------------------------
# Initialize variables

MAX_SPEED = 6.28

# Use Supervisor instead of Robot to access world objects
robot = Supervisor()

timestep = int(robot.getBasicTimeStep())
delta_t = robot.getBasicTimeStep()/1000.0

# Add new state
states = ['search_wall', 'follow_wall', 'turn_right','turn_left', 'curve_+90', 'move_to_pipe', 'reached_pipe']
current_state = 'search_wall'

# Distance threshold to consider pipe "reached"
PIPE_REACHED_DISTANCE = 0.1  # meters

counter = 0
COUNTER_MAX = 100

# Kinematics
R = 0.0205
D = 0.0520

# Gains
kd = 0.003
kd2 = 0.003
d_desired = 200

#-------------------------------------------------------
# Devices

ps = []
psNames = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)

encoder = []
encoderNames = ['left wheel sensor','right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)

oldEncoderValues = []

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# Get the pipe node using supervisor
pipe_node = robot.getFromDef('pipe(1)')
if pipe_node is None:
    # Try alternative names
    pipe_node = robot.getFromDef('PIPE')
if pipe_node is None:
    pipe_node = robot.getFromDef('Pipe')

############################################################################
# FUNCTIONS
############################################################################

def get_wheels_speed(enc, old, dt):
    wl = (enc[0] - old[0]) / dt
    wr = (enc[1] - old[1]) / dt
    return wl, wr

def get_robot_speeds(wl, wr, r, d):
    u = r * (wl + wr) / 2.0
    w = r * (wr - wl) / d
    return u, w

def wheel_speed_commands(u_d, w_d, D, R):
    wl = (u_d / R) - (w_d * D) / (2 * R)
    wr = (u_d / R) + (w_d * D) / (2 * R)

    # Clamp speeds
    max_mag = max(abs(wl), abs(wr), 1e-9)
    if max_mag > MAX_SPEED:
        scale = MAX_SPEED / max_mag
        wl *= scale
        wr *= scale

    return wl, wr


def follow_wall_to_left(kd, kd2, d_l, d_fl, d_desired):
    error = d_desired - d_l
    angle_error = d_fl - d_l

    w_ref = kd * error + kd2 * angle_error
    w_ref = max(min(w_ref, 0.4), -0.4)

    U_MAX = 0.04
    u_ref = U_MAX / (1 + abs(w_ref) * 4)

    if u_ref < 0.01:
        u_ref = 0.01

    return u_ref, w_ref


def detect_pipe_supervisor(robot, pipe_node):
    """Detects pipe position relative to robot using supervisor."""
    if pipe_node is None:
        return False, 0, 0
    
    # Get robot position and orientation
    robot_node = robot.getSelf()
    robot_pos = robot_node.getPosition()
    robot_rot = robot_node.getOrientation()
    
    # Get pipe position
    pipe_pos = pipe_node.getPosition()
    
    # Calculate relative position
    dx = pipe_pos[0] - robot_pos[0]
    dy = pipe_pos[1] - robot_pos[1]
    
    # Distance to pipe
    distance = math.sqrt(dx*dx + dy*dy)
    
    # Only detect if within range (e.g., 1.5 meters)
    if distance > 0.6:
        return False, 0, 0
    
    # Calculate angle to pipe relative to robot's heading
    # Robot's forward direction from rotation matrix
    forward_x = robot_rot[0]
    forward_y = robot_rot[3]
    
    # Angle to pipe
    angle_to_pipe = math.atan2(dy, dx)
    robot_heading = math.atan2(forward_y, forward_x)
    
    # Relative angle (-pi to pi)
    relative_angle = angle_to_pipe - robot_heading
    while relative_angle > math.pi:
        relative_angle -= 2 * math.pi
    while relative_angle < -math.pi:
        relative_angle += 2 * math.pi
    
    # Only detect if pipe is roughly in front (within ~90 degrees)
    if abs(relative_angle) > math.pi / 2:
        return False, 0, 0
    
    return True, relative_angle, distance


def move_towards_pipe(relative_angle, distance):
    """Generates speed commands to move towards the detected pipe."""
    kp_turn = 1.5
    
    # Slow down as we get closer
    if distance < 0.1:
        u_ref = 0.01
    elif distance < 0.3:
        u_ref = 0.03
    else:
        u_ref = 0.08
    
    # Turn towards pipe
    w_ref = kp_turn * relative_angle
    w_ref = max(min(w_ref, 0.6), -0.6)
    
    return u_ref, w_ref


############################################################################
# MAIN LOOP
############################################################################

while robot.step(timestep) != -1:

    # ------------------ SEE ---------------------
    psValues = [ps[i].getValue() for i in range(8)]
    encoderValues = [encoder[i].getValue() for i in range(2)]

    if len(oldEncoderValues) < 2:
        oldEncoderValues = encoderValues.copy()

    # Check for pipe detection using supervisor
    pipe_detected, pipe_angle, pipe_distance = detect_pipe_supervisor(robot, pipe_node)

    # ------------------ THINK --------------------
    u_d = 0.03
    w_d = 0.0

    # Priority: If pipe is detected, switch to move_to_pipe state
    if pipe_detected and current_state != 'move_to_pipe' and current_state != 'reached_pipe':
        current_state = 'move_to_pipe'
    
    # If close enough to pipe, stop
    if current_state == 'move_to_pipe' and pipe_detected and pipe_distance < PIPE_REACHED_DISTANCE:
        current_state = 'reached_pipe'
    
    # If we were tracking pipe but lost it, go back to search_wall
    if current_state == 'move_to_pipe' and not pipe_detected:
        current_state = 'search_wall'

    # ------------ ACTIONS -------------
    if current_state == 'reached_pipe':
        u_d = 0.0
        w_d = 0.0

    elif current_state == 'move_to_pipe':
        u_d, w_d = move_towards_pipe(pipe_angle, pipe_distance)

    elif current_state == 'search_wall':
        u_d = 0.05
        w_d = 0.0

    elif current_state == 'follow_wall':
        u_d, w_d = follow_wall_to_left(kd, kd2, psValues[5], psValues[6], d_desired)

    elif current_state == 'turn_right':
        u_d = 0.0
        w_d = -0.5

    elif current_state == 'turn_left':
        u_d = 0.0
        w_d = 0.5    

    elif current_state == 'curve_+90':
        u_d = 0.03
        w_d = 0.6

    # ------------ TRANSITIONS ------------
    if current_state == 'search_wall':
        if psValues[5] > 100 or psValues[6] > 100:
            current_state = 'follow_wall'

    elif current_state == 'follow_wall':
        if psValues[5] < 80 and psValues[6] < 80:
            if psValues[0] < 80 or psValues[7] < 80:
                current_state = 'search_wall'
            else:
                current_state = 'curve_+90'
            counter = 20

        if psValues[0] > 150 or psValues[7] > 150:
            if psValues[2]< psValues[5]:
                current_state = 'turn_right'

            else: 
                current_state == 'turn_left'
            counter = 0 

    elif current_state == 'curve_+90':
        if counter >= COUNTER_MAX:
            current_state = 'follow_wall'

    elif current_state == 'turn_right' or current_state == 'turn_left':
        if counter >= COUNTER_MAX:
            current_state = 'follow_wall'

    counter += 1
    oldEncoderValues = encoderValues.copy()

    # ------------------ ACT ---------------------
    wl_d, wr_d = wheel_speed_commands(u_d, w_d, D, R)
    leftMotor.setVelocity(wl_d)
    rightMotor.setVelocity(wr_d)

    pipe_status = f"PIPE (angle:{pipe_angle:.2f}, dist:{pipe_distance:.2f})" if pipe_detected else ""
    print(f"STATE={current_state} ps5={psValues[5]:.1f} ps6={psValues[6]:.1f} ps0={psValues[0]:.1f} {pipe_status}")