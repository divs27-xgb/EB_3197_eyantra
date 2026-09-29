# =============================================================================
#  Task 1C  -  Balancing the Rotary Inverted Pendulum (Furuta pendulum)
#  CoppeliaSim child script  -  BOILERPLATE
# =============================================================================
#
#  WHERE THIS GOES
#  ---------------
#  This is the child script attached to the "Motor" object in task_1c.ttt.
#  Click the script icon next to Motor in the scene hierarchy to open it.
#
#
#  WHAT YOU HAVE TO BUILD
#  ----------------------
#  Two things, at the same time:
#     1. hold the PENDULUM UPRIGHT          ->  tilt  stays near 0
#     2. hold the ARM STILL                 ->  yaw   stops drifting away
#
#  Keeping the pendulum up while the arm spins away forever does NOT count.
#
#
#  THE STATE VECTOR  (this is the Task 1B one - do not reorder it)
#  ---------------------------------------------------------------
#
#        x = [ tilt_dot ,  tilt ,  yaw_dot ,  yaw ]
#              ^rate       ^angle  ^rate     ^angle
#              |           |       |         |
#              |           |       |         +-- arm angle        (Task 1B: "yaw")
#              |           |       +------------ arm speed
#              |           +-------------------- pendulum angle   (Task 1B: "tilt")
#              +-------------------------------- pendulum speed
#
#  RATE first, then ANGLE.  TILT before YAW.
#
#  This is the same order the Task 1B LQR Tuner printed at the top of its
#  window, and the same order your A, B and Q were written in. The four
#  numbers in your gain K were computed against THIS order.
#
#     >>> A correct K applied to a reordered x is a WRONG controller,
#     >>> and it fails looking exactly like badly tuned gains. <<<
#
#
#  UNITS
#  -----
#  Everything crossing the CoppeliaSim API is in metres, kilograms, seconds
#  and RADIANS. The angles you read below are already radians, so your gains
#  must expect radians too.
#
#
#  WHAT TASK 1B DID FOR YOU, THAT YOU NOW DO YOURSELF
#  ---------------------------------------------------
#     Task 1B window            ->   here
#     ------------------------------------------------------------------
#     read tilt / yaw           ->   sysCall_sensing()   (STEP 2)
#     applied your gains        ->   sysCall_actuation() (STEP 3)
#     ran the loop at the rate  ->   dt, read in sysCall_init()
#     "Toggle Motor sign"       ->   MOTOR_SIGN below
#     "Zero integrals"          ->   nothing. You handle wind-up.
#     "Reset now"               ->   nothing. It falls, the run is over.
#
#
#  THE ORDER COPPELIASIM CALLS THESE  (this surprises people)
#  -----------------------------------------------------------
#  Within ONE simulation step:
#
#        sysCall_actuation()   <-- runs FIRST
#            (physics is stepped)
#        sysCall_sensing()     <-- runs SECOND
#
#  So the state values your control law uses in sysCall_actuation() were
#  measured by sysCall_sensing() on the PREVIOUS step. That is normal, and
#  it is why the four states live in global variables instead of being
#  handed straight from one function to the other.
#
#  The STEP numbers below are the order it is easiest to WRITE them in,
#  not the order they run.
#
#
#  HOW TO USE THIS FILE
#  --------------------
#  Work through the STEP 1..4 markers in order. Everything already written is
#  plumbing you can keep as-is. Everything marked
#
#        ####### ADD YOUR CODE HERE ######
#
#  is yours to write.
#
# =============================================================================


###### GLOBAL VARIABLES HERE ######

# --- Scene object handles (filled in by sysCall_init) ---------------------
base     = None
motor    = None          # the actuated joint - your ONLY input
arm      = None
pendulum = None
elbow    = None          # the free joint the pendulum hangs on

# --- The four states, refreshed every step by sysCall_sensing -------------
tilt      = 0.0          # pendulum angle, measured FROM UPRIGHT   [rad]
tilt_dot  = 0.0          # pendulum angular velocity               [rad/s]
yaw       = 0.0          # arm angle                               [rad]
yaw_dot   = 0.0          # arm angular velocity                    [rad/s]

# --- The control output ---------------------------------------------------
U  = 0.0                 # target velocity sent to the motor       [rad/s]
dt = 0.0                 # length of one simulation step           [s]

# --- Motor direction ------------------------------------------------------
# This is the "Toggle Motor sign" button from Task 1B, frozen into a number.
# If the pendulum falls over instantly and hard no matter what your gains
# are, CHANGE THIS TO -1 BEFORE YOU TOUCH ANYTHING ELSE.
MOTOR_SIGN = +1

# --- Safety limit ---------------------------------------------------------
# The motor cannot deliver unlimited speed. Decide what your code should do
# when the control law asks for more than this.
U_MAX = 20.0             # [rad/s]  tune to taste

# ---------------------------------------------------------------------------
#  YOUR GAINS
#  Copy in what already balanced the pendulum in Task 1B. Use ONE of these
#  two blocks - you do not need both.
# ---------------------------------------------------------------------------

# ---- Option A: LQR -------------------------------------------------------
# The row of four numbers the Task 1B LQR Tuner printed when you pressed
# COMPUTE LQR (or Print K). Same order as x above.
#
#     K = [ k_tilt_dot , k_tilt , k_yaw_dot , k_yaw ]
#K = [-14.9049, -140.6569, -13.5089, -27.5653]
K=[-25.0566 ,-250.213 ,-32.3111 ,-78.0752]

# ---- Option B: PID -------------------------------------------------------
# The six slider values from the Task 1B PID Tuner (Print gains).
Kp_tilt, Ki_tilt, Kd_tilt = 0.0, 0.0, 0.0
Kp_yaw,  Ki_yaw,  Kd_yaw  = 0.0, 0.0, 0.0

# PID needs memory between steps. These are what "Zero integrals" used to clear.
integral_tilt = 0.0
integral_yaw  = 0.0

# You can add variables here
# as required by your implementation.
###################################


# =============================================================================
#  Small helpers - already written for you, use them if they are useful.
# =============================================================================

def wrap_to_pi(angle):
    """Fold any angle into the range -pi .. +pi.

    A joint that has spun several turns reports a large angle. Without this,
    an error of 'a bit past 180 degrees' looks like a huge error and your
    controller lurches the wrong way.
    """
    import math
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value, limit):
    """Keep value inside -limit .. +limit."""
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


# =============================================================================
def sysCall_init():
    # This function runs ONCE, when the simulation starts.

    global sim
    global base, motor, arm, pendulum, elbow
    global dt, integral_tilt, integral_yaw

    # CoppeliaSim 4.6+ hands the API to Python scripts through require().
    sim = require('sim')

    # ---------------------------------------------------------------------
    #  STEP 1  -  Grab the scene objects.
    #             Done for you. The names match the scene hierarchy exactly;
    #             changing them in the scene will break this.
    # ---------------------------------------------------------------------
    base     = sim.getObject('/Base')
    motor    = sim.getObject('/Motor')
    arm      = sim.getObject('/Arm')
    pendulum = sim.getObject('/Pendulum')
    elbow    = sim.getObject('/Elbow')

    # How long one simulation step lasts. Your integral and derivative terms
    # need this. Take it from the simulator - never hard-code it.
    dt = sim.getSimulationTimeStep()

    # Start every run with a clean integrator.
    integral_tilt = 0.0
    integral_yaw  = 0.0

    ####### ADD YOUR CODE HERE ######
    # Anything else your controller needs set up once, before the first step.
    # For example: variables for a derivative term, or a counter.

    #################################
    pass


# =============================================================================
def sysCall_sensing():
    # This function runs at EVERY simulation step.
    # Read the world here. Do not command the motor from this function.

    global tilt, tilt_dot, yaw, yaw_dot

    ####### ADD YOUR CODE HERE ######
    #
    #  STEP 2  -  Fill in the four states.
    #
    #  These are the numbers the Task 1B window showed you under "Live values"
    #  as `tilt [rad]` and `yaw [rad]`. Now you fetch them yourself.
    #
    #  The two API calls you need:
    #
    #        sim.getJointPosition(handle)   -> angle of that joint    [rad]
    #        sim.getJointVelocity(handle)   -> speed of that joint    [rad/s]
    #
    #  Which handle for which state?
    #        pendulum tilt / tilt_dot  ->  the FREE joint  (elbow)
    #        arm      yaw  / yaw_dot   ->  the MOTOR joint (motor)
    #
    #  TWO THINGS THAT CATCH PEOPLE OUT:
    #
    #  (a) MEASURED FROM WHERE?
    #      Your model was linearised about UPRIGHT, so `tilt` must be the
    #      angle away from straight up, and must be 0 when balanced.
    #      If the joint reports 0 when the pendulum is HANGING instead,
    #      you have to convert. Print the raw value with the pendulum
    #      hanging and again with it upright, and see which you have.
    #
    #  (b) WRAPPING
    #      Consider passing angles through wrap_to_pi() so a joint that has
    #      spun right round does not produce a nonsense error.
    #
    #  Sketch:
    #        tilt     = ...          # from the free joint, zero = upright
    #        tilt_dot = ...
    #        yaw      = ...          # from the motor joint
    #        yaw_dot  = ...
    tilt=sim.getJointPosition(elbow)
    tilt_dot=sim.getJointVelocity(elbow)
    yaw=sim.getJointPosition(motor)
    yaw_dot=sim.getJointVelocity(motor)

    #################################
    pass


# =============================================================================
def sysCall_actuation():
    # This function runs at EVERY simulation step.
    # Turn the four states into ONE number and send it to the motor.

    global U, integral_tilt, integral_yaw

    ####### ADD YOUR CODE HERE ######
    #
    #  STEP 3  -  Compute U, then send it.
    #
    #  Use ONE of the two approaches below. Delete or ignore the other.
    #
    #  ---------------------------------------------------------------------
    #  OPTION A - LQR
    #  ---------------------------------------------------------------------
    #  One line of control law, applied every step:
    #
    #        u = -K x
    #
    #  written out, and paying attention to the ORDER:
    #
    #        U = -( K[0]*tilt_dot + K[1]*tilt + K[2]*yaw_dot + K[3]*yaw )
    #
    #  Note all four states appear. Feed it only the two angles and the
    #  other two gains silently do nothing.
    U=-(K[0]*tilt_dot+K[1]*tilt+K[2]*yaw_dot+K[3]*yaw)
    #
    #  ---------------------------------------------------------------------
    #  OPTION B - PID
    #  ---------------------------------------------------------------------
    #  Two loops, exactly like the two sets of sliders in Task 1B, summed
    #  into one motor command ("parallel PID").
    #
    #    error_tilt = 0.0 - tilt          # you want tilt at 0 (upright)
    #    error_yaw  = 0.0 - yaw           # you want yaw held where it is
    #
    #    integral_tilt = integral_tilt + error_tilt * dt
    #    integral_yaw  = integral_yaw  + error_yaw  * dt
    #
    #    # For the D term, the joint's reported velocity is usually steadier
    #    # than differencing the angle yourself.
    #
    #    u_tilt = Kp_tilt*error_tilt + Ki_tilt*integral_tilt - Kd_tilt*tilt_dot
    #    u_yaw  = Kp_yaw *error_yaw  + Ki_yaw *integral_yaw  - Kd_yaw *yaw_dot
    #    U = u_tilt + u_yaw
    #
    #    WIND-UP: while the pendulum is far from upright, error piles up in
    #    integral_* and stays there. Task 1B had a "Zero integrals" button.
    #    You do not. Consider clamping the integral, or leaving Ki at 0.
    #
    #  ---------------------------------------------------------------------
    #  THEN, WHICHEVER YOU CHOSE
    #  ---------------------------------------------------------------------
    #    U = clamp(U, U_MAX)              # respect the motor's limit
    #    U = MOTOR_SIGN * U               # the Task 1B motor-sign button
    #
    #    sim.setJointTargetVelocity(motor, U)
    #
    #    (Motor is a revolute joint in VELOCITY CONTROL mode, so what you
    #     send it is a target velocity - the `U (motor vel.)` readout you
    #     watched in Task 1B.)
    U=clamp(U,U_MAX)
    U=MOTOR_SIGN * U
    sim.setJointTargetVelocity(motor,U)

    #################################
    pass


# =============================================================================
def sysCall_cleanup():
    # This function runs ONCE, when the simulation ends.

    ####### ADD YOUR CODE HERE ######
    #
    #  STEP 4  -  Leave the scene as you found it.
    #
    #  Optional, but good practice: stop the motor so a failed run does not
    #  leave the scene in a strange state.
    #
    #        sim.setJointTargetVelocity(motor, 0.0)
    sim.setJointTargetVelocity(motor,0.0)

    #################################
    pass


# =============================================================================
#  STUCK? WORK THROUGH THIS BEFORE RE-TUNING.
# =============================================================================
#
#  Your gains already balanced this pendulum once, in Task 1B. So when it
#  falls here, the loop around them is usually at fault - not the numbers.
#
#    Falls instantly and hard, whatever the gains
#         -> MOTOR_SIGN. Set it to -1 and run again.
#
#    Fails like bad tuning, but those gains worked in 1B
#         -> state ORDER. K expects [tilt_dot, tilt, yaw_dot, yaw].
#
#    Holds a moment, then slides away smoothly
#         -> tilt is measured from the wrong zero (hanging, not upright).
#
#    Pendulum stays up but the arm rotates away forever
#         -> no yaw term, or far too weak. That is what Kp (yaw) was for.
#
#    Jitters violently around upright
#         -> D term amplifying noise, or dt hard-coded instead of read.
#
#    Fine at first, then a slow growing wobble
#         -> integral wind-up. Nothing zeroes it for you here.
#
#  SANITY-CHECK FIRST. Before touching a single gain, print your four states
#  each step and confirm they look like Task 1B's "Live values" did:
#
#        print(f"tilt={tilt:+.3f} tilt_dot={tilt_dot:+.3f} "
#              f"yaw={yaw:+.3f} yaw_dot={yaw_dot:+.3f} U={U:+.3f}")
#
#  tilt near 0 when upright, yaw tracking where the arm points, both in
#  radians. If those numbers are wrong, no gain will save you.
#
# =============================================================================
# See the user manual or the available code snippets for additional callback
# functions and details.
