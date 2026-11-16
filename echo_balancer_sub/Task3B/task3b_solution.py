# =============================================================================
#  Task 3B  -  CoppeliaSim child script  -  BOILERPLATE
# =============================================================================
#  Team ID          : 3197
#  Team members     : 1.Devesh dev
#                     2.Kartik bhardwaj
#                     3.Priyansh singh rathore
#
# =============================================================================
#  Write your code only where it says
#
#        ####### WRITE YOUR LOGIC HERE ######
#
#  Do not use sim.getSimulatorMessage() anywhere in this script.

import math

###### GLOBAL VARIABLES ######
# You can add your own variables here.

##############################
reference_frame = None
front_motor = None  # drives the omni wheel: your ONLY input
drive_motor = None
front_sensor = None
right_sensor = None
left_sensor = None

tilt = 0.0
tilt_dot = 0.0
yaw = 0.0
yaw_dot = 0.0

U = 0.0
dt = 0.01
dist = [0.0, 0.0, 0.0]
sensor_hit = [False, False, False]
obj_hit = [-1, -1, -1]
front_normal = None
MOTOR_SIGN = -1
sensors_ready = False

Kp_tilt, Ki_tilt, Kd_tilt = 70.0, 0.0, 14.0
Kp_yaw, Ki_yaw, Kd_yaw = 6.5, 0.0, 20
tilt_integral = 0.0
yaw_integral = 0.0
YAW_LIMIT = 1
YAW_SLEW = 0.7

# ---- speed profile ----
MAX_SPEED = 60.0
APPROACH_SPEED = 12.0
SLOW_START_DIST = 0.55
TURN_TRIGGER_DIST = 0.25
STOP_DIST = 0.06

# ---- turn detection ----
TURN90_TRIGGER_DIST = 0.2
BEND_TRIGGER_DIST = 0.45
TURN_REARM_DIST = 0.4
BEND_ANGLE = math.pi / 4.0
NORMAL_SLANT_MIN = 0.35
NORMAL_CONFIRM = 5
slant_count = 0

# ---- turn completion ----
TURN_DONE_YAW_ERR = 0.05
TURN_DONE_YAW_RATE = 0.3
TURN_TIMEOUT_STEPS = 800

# ---- finish / stop ----
STOP_CONFIRM_SAMPLES = 40
STOP_DECEL = 0.5

# ---- state ----
turn_pending = False
turn_armed = True
bend_armed = True
turn_mode = ''
turn_steps = 0
INITIAL_PATH_HEADING = 0.0
path_heading = 0.0
yaw_setpoint = 0.0
yaw_ref = 0.0
allout_count = 0
seen_wall = False
speed = -60
stop = 0


def wrap_to_pi(angle):
    """Fold any angle into -pi .. +pi"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, limit):
    return max(-limit, min(limit, value))


def bike_tilt_yaw():
    m = sim.getObjectMatrix(drive_motor, reference_frame)
    axle = [m[2], m[6], m[10]]

    front = sim.getObjectPosition(front_motor, reference_frame)
    rear = sim.getObjectPosition(drive_motor, reference_frame)
    forward = [front[i] - rear[i] for i in range(3)]
    length = math.sqrt(sum(v * v for v in forward))
    forward = [v / length for v in forward]

    up_z = axle[0] * forward[1] - axle[1] * forward[0]

    t = math.atan2(-axle[2], up_z)
    y = math.atan2(-forward[0], forward[1])
    return t, y


def calc_speed():
    """Drive speed on a straight: full speed, smooth slow-down as the front wall approaches."""
    if not sensor_hit[1]:
        return -MAX_SPEED
    if dist[1] <= STOP_DIST:
        return 0.0
    if dist[1] <= TURN_TRIGGER_DIST:
        return -APPROACH_SPEED
    if dist[1] >= SLOW_START_DIST:
        return -MAX_SPEED

    span = max(SLOW_START_DIST - TURN_TRIGGER_DIST, 1e-6)
    ratio = clamp((dist[1] - TURN_TRIGGER_DIST) / span, 1.0)
    smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
    return -(APPROACH_SPEED + (MAX_SPEED - APPROACH_SPEED) * smooth_ratio)


def start_turn(angle, mode, label):
    """Shift the absolute corridor heading by an exact angle and start the turn."""
    global path_heading, yaw_setpoint, turn_pending, turn_armed, bend_armed, turn_mode, turn_steps
    path_heading = wrap_to_pi(path_heading + angle)
    yaw_setpoint = path_heading
    turn_pending = True
    bend_armed = False
    if mode == '90':
        turn_armed = False    # a 45 deg bend leaves the 90 deg trigger armed (back-to-back corners)
    turn_mode = mode
    turn_steps = 0
    print(f"TURN {label} t={sim.getSimulationTime():.2f} front={dist[1]:.3f} "
          f"left={dist[0]:.3f} right={dist[2]:.3f}")


def sysCall_init():
    # Runs ONCE, when the simulation starts.
    global sim, reference_frame, front_motor, drive_motor, dt, front_sensor, right_sensor, left_sensor
    global tilt, yaw, stop, turn_pending, turn_armed, bend_armed, turn_mode
    global path_heading, yaw_setpoint, yaw_ref, turn_steps, allout_count, seen_wall, slant_count

    sim = require('sim')

    reference_frame = sim.getObject('/reference_frame')
    front_motor = sim.getObject('/reference_frame/spherical_joint/bike_respondable/front_motor')
    drive_motor = sim.getObject('/reference_frame/spherical_joint/bike_respondable/drive_motor')

    front_sensor = sim.getObject("/FrontSensor")
    left_sensor = sim.getObject("/LeftSensor")
    right_sensor = sim.getObject("/RightSensor")
    stop = 0
    turn_pending = False
    turn_armed = True
    bend_armed = True
    turn_mode = ''

    dt = sim.getSimulationTimeStep()

    # First reading, so the first tilt_dot and yaw_dot are not a jump
    tilt, yaw = bike_tilt_yaw()

    ####### WRITE YOUR LOGIC HERE ######
    path_heading = yaw if INITIAL_PATH_HEADING is None else INITIAL_PATH_HEADING
    yaw_setpoint = path_heading
    yaw_ref = path_heading
    turn_steps = 0
    allout_count = 0
    seen_wall = False
    slant_count = 0
    ####################################


def sysCall_sensing():
    # Runs every simulation step. Read the world here.
    global tilt, tilt_dot, yaw, yaw_dot, dist, sensor_hit, sensors_ready, obj_hit, front_normal
    global allout_count, seen_wall

    ####### WRITE YOUR LOGIC HERE ######
    new_tilt, new_yaw = bike_tilt_yaw()

    tilt_dot = (new_tilt - tilt) / dt
    yaw_dot = wrap_to_pi(new_yaw - yaw) / dt

    tilt = new_tilt
    yaw = new_yaw

    result_f, distance_f, point_f, obj_f, normal_f = sim.readProximitySensor(front_sensor)
    result_r, distance_r, point_r, obj_r, normal_r = sim.readProximitySensor(right_sensor)
    result_l, distance_l, point_l, obj_l, normal_l = sim.readProximitySensor(left_sensor)
    dist = [distance_l, distance_f, distance_r]
    sensor_hit = [bool(result_l), bool(result_f), bool(result_r)]
    obj_hit = [obj_l, obj_f, obj_r]
    front_normal = list(normal_f) if result_f else None

    # Finish detection: count consecutive samples where NO sensor sees anything
    if any(sensor_hit):
        seen_wall = True
        allout_count = 0
    else:
        allout_count += 1

    sensors_ready = True
    ####################################
    pass


def sysCall_actuation():
    # Runs every simulation step. Set the motors here.
    global U, tilt_integral, yaw_integral, yaw_setpoint, yaw_ref, speed
    global turn_pending, turn_armed, bend_armed, turn_mode, turn_steps
    global stop, slant_count

    ####### WRITE YOUR LOGIC HERE ######
    if not sensors_ready:
        speed = 0.0
    elif not stop:

        # finish: all sensors clear long enough -> stop
        if seen_wall and allout_count >= STOP_CONFIRM_SAMPLES and not turn_pending:
            stop = 1
            print(f"STOP t={sim.getSimulationTime():.2f}")

        #  re-arm turn detection once the front is clear
        if not turn_pending and (not sensor_hit[1] or dist[1] > TURN_REARM_DIST):
            turn_armed = True
            bend_armed = True

        # front-wall orientation from the surface normal (sensor frame)
        # Perpendicular wall: normal ~ (0, 0, -1). 45 deg wall: |nx| ~ 0.71, nx < 0 -> wall faces left.
        nx = front_normal[0] if (sensor_hit[1] and front_normal) else 0.0
        if abs(nx) >= NORMAL_SLANT_MIN:
            sgn = 1 if nx > 0 else -1
            slant_count = slant_count + sgn if slant_count * sgn >= 0 else sgn
        else:
            slant_count = 0
        slanted = abs(slant_count) >= NORMAL_CONFIRM

        # 90 deg pivot: perpendicular wall ahead, one side open
        if (turn_armed and sensor_hit[1] and dist[1] <= TURN90_TRIGGER_DIST and not slanted
                and dist[0] > 0.0 and dist[2] == 0.0 and not turn_pending):
            start_turn(-math.pi / 2.0, '90', '90R')
        elif (turn_armed and sensor_hit[1] and dist[1] <= TURN90_TRIGGER_DIST and not slanted
                and dist[0] == 0.0 and dist[2] > 0.0 and not turn_pending):
            start_turn(math.pi / 2.0, '90', '90L')

        if turn_pending:
            speed = -APPROACH_SPEED if turn_mode == '45' else 0.0
        else:
            speed = calc_speed()

        # 45 deg bend: slanted wall ahead
        if (bend_armed and not turn_pending and sensor_hit[1]
                and dist[1] <= BEND_TRIGGER_DIST and slanted):
            if nx < 0.0:
                start_turn(BEND_ANGLE, '45', '45L')
            else:
                start_turn(-BEND_ANGLE, '45', '45R')

        #  turn completion
        if turn_pending:
            turn_steps += 1
            err_abs = abs(wrap_to_pi(yaw - yaw_setpoint))
            settled = err_abs <= TURN_DONE_YAW_ERR and abs(yaw_dot) <= TURN_DONE_YAW_RATE
            if settled or turn_steps >= TURN_TIMEOUT_STEPS:
                turn_pending = False
                turn_mode = ''

    elif stop:
        speed += clamp(0.0 - speed, STOP_DECEL)   # controlled deceleration
        yaw_setpoint = path_heading

    #  yaw reference--- rate-limited for 90 deg pivots, step for 45 deg bends
    target = wrap_to_pi(yaw_setpoint)
    if turn_mode == '45':
        yaw_ref = target
    else:
        yaw_ref = wrap_to_pi(yaw_ref + clamp(wrap_to_pi(target - yaw_ref), YAW_SLEW * dt))
    yaw_offset = wrap_to_pi(yaw - yaw_ref)
    yaw_offset = clamp(yaw_offset, YAW_LIMIT)

    #  PID:
    tilt_integral += tilt * dt
    yaw_integral += yaw_offset * dt
    U = (Kp_tilt * tilt + Ki_tilt * tilt_integral + Kd_tilt * tilt_dot) \
        - (Kp_yaw * yaw_offset + Ki_yaw * yaw_integral + Kd_yaw * yaw_dot)

    sim.setJointTargetVelocity(front_motor, -MOTOR_SIGN * U)
    sim.setJointTargetVelocity(drive_motor, speed)
    ####################################
    pass


def sysCall_cleanup():
    # Runs ONCE, when the simulation ends.

    ####### WRITE YOUR LOGIC HERE ######
    sim.setJointTargetVelocity(front_motor, 0)
    ####################################
    pass
