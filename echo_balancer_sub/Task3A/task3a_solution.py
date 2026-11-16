# =============================================================================
#  Task 3A  -  CoppeliaSim child script  -  BOILERPLATE
# =============================================================================
#  Team ID          : EB_3197
#  Team members     : 1.Devesh dev
#                     2.Kartik bhardwaj
#                     3.Priyansh singh rathore
#                     4.
# =============================================================================
#  Write your code only where it says
#
#        ####### WRITE YOUR LOGIC HERE ######
#
#  Do not use sim.getSimulatorMessage() anywhere in this script.
# =============================================================================
import math

# balance (tilt) and heading (yaw) gains
Kp_tilt, Kd_tilt = 42.0, 9.935
Kp_yaw, Kd_yaw = 5.8, 18.65
YAW_LIMIT = 0.4          # max yaw error fed to the controller, keeps turns gentle

# driving
MAX_SPEED = 60.0
SPEED_SLEW = 300.0
SLOW_START_DIST = 0.6
TURN_DIST = 0.2
APPROACH_MARGIN = 0.05

TURN_DONE_ERR = 0.05
TURN_DONE_RATE = 0.2

# braking at the end
BRAKE_K = 0.25
BRAKE_MAX = 6.0
BRAKE_SLEW = 120.0
STOP_SPEED = 1.0


def wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, limit):
    return max(-limit, min(limit, value))


def get_tilt_yaw():
    # rear axle direction and the rear->front vector give us lean and heading
    m = sim.getObjectMatrix(drive_motor, reference_frame)
    axle = [m[2], m[6], m[10]]

    front = sim.getObjectPosition(front_motor, reference_frame)
    rear = sim.getObjectPosition(drive_motor, reference_frame)
    fwd = [front[i] - rear[i] for i in range(3)]
    length = math.sqrt(sum(v * v for v in fwd))
    fwd = [v / length for v in fwd]

    up_z = axle[0] * fwd[1] - axle[1] * fwd[0]
    tilt = math.atan2(-axle[2], up_z)
    yaw = math.atan2(-fwd[0], fwd[1])
    return tilt, yaw


def sysCall_init():
    global sim, reference_frame, front_motor, drive_motor, front_sensor, left_sensor, right_sensor
    global dt, tilt, yaw, tilt_dot, yaw_dot, yaw_setpoint, speed
    global wall, front_dist, sensors_ready, path_seen, turning, stopped

    sim = require('sim')

    reference_frame = sim.getObject('/reference_frame')
    front_motor = sim.getObject('/reference_frame/spherical_joint/bike_respondable/front_motor')
    drive_motor = sim.getObject('/reference_frame/spherical_joint/bike_respondable/drive_motor')
    front_sensor = sim.getObject('/FrontSensor')
    left_sensor = sim.getObject('/LeftSensor')
    right_sensor = sim.getObject('/RightSensor')

    dt = sim.getSimulationTimeStep()
    tilt, yaw = get_tilt_yaw()
    tilt_dot = 0.0
    yaw_dot = 0.0
    yaw_setpoint = 0.0
    speed = 0.0

    wall = [False, False, False]     # left, front, right
    front_dist = 0.0
    sensors_ready = False
    path_seen = False
    turning = False
    stopped = False


def sysCall_sensing():
    global tilt, yaw, tilt_dot, yaw_dot, wall, front_dist, sensors_ready

    new_tilt, new_yaw = get_tilt_yaw()
    tilt_dot = (new_tilt - tilt) / dt
    yaw_dot = wrap_to_pi(new_yaw - yaw) / dt
    tilt, yaw = new_tilt, new_yaw

    hit_l = sim.readProximitySensor(left_sensor)[0]
    hit_f, front_dist = sim.readProximitySensor(front_sensor)[:2]
    hit_r = sim.readProximitySensor(right_sensor)[0]
    wall = [bool(hit_l), bool(hit_f), bool(hit_r)]
    sensors_ready = True


def target_speed():
    # full speed until the front wall gets close, then ease off
    if not wall[1] or front_dist >= SLOW_START_DIST:
        return -MAX_SPEED
    zero_dist = TURN_DIST - APPROACH_MARGIN
    ratio = (front_dist - zero_dist) / (SLOW_START_DIST - zero_dist)
    return -MAX_SPEED * max(0.0, min(1.0, ratio)) ** 1.5


def sysCall_actuation():
    global yaw_setpoint, speed, path_seen, turning, stopped

    ####### WRITE YOUR LOGIC HERE ######
    if not sensors_ready:
        speed = 0.0
    elif not stopped:
        if any(wall):
            path_seen = True

        # wall ahead and only one side open -> turn 90 deg towards the open side
        if not turning and wall[1] and front_dist < TURN_DIST and wall[0] != wall[2]:
            direction = 1.0 if not wall[0] else -1.0     # +1 = left, -1 = right
            print("turning", "left" if direction > 0 else "right")
            yaw_setpoint = wrap_to_pi(yaw_setpoint + direction * math.pi / 2)
            turning = True

        if turning:
            speed = 0.0
            if abs(wrap_to_pi(yaw - yaw_setpoint)) <= TURN_DONE_ERR and abs(yaw_dot) <= TURN_DONE_RATE:
                turning = False
        else:
            speed += clamp(target_speed() - speed, SPEED_SLEW * dt)

        # no walls anywhere after we've been in the track -> we're past the finish
        if path_seen and not any(wall):
            stopped = True
            yaw_setpoint = yaw

    if stopped:
        wheel_speed = sim.getJointVelocity(drive_motor)
        brake = 0.0 if abs(wheel_speed) <= STOP_SPEED else clamp(-BRAKE_K * wheel_speed, BRAKE_MAX)
        speed += clamp(brake - speed, BRAKE_SLEW * dt)

    # PD control: balance the bike and hold the heading
    yaw_err = clamp(wrap_to_pi(yaw - yaw_setpoint), YAW_LIMIT)
    u = (Kp_tilt * tilt + Kd_tilt * tilt_dot) - (Kp_yaw * yaw_err + Kd_yaw * yaw_dot)

    sim.setJointTargetVelocity(front_motor, u)
    sim.setJointTargetVelocity(drive_motor, speed)


def sysCall_cleanup():
    sim.setJointTargetVelocity(front_motor, 0)
    sim.setJointTargetVelocity(drive_motor, 0)
