from controller import Robot


# Constants

TIME_STEP = 64
MAX_SPEED = 6.28

# Distance sensor thresholds (tune!)
FRONT_OBSTACLE_THRESHOLD = 80
WALL_DISTANCE_THRESHOLD = 60

# Light detection threshold (tune!)
LIGHT_THRESHOLD = 450

# States
FOLLOW_LEFT = 0
AVOID_OBSTACLE = 1
TURN_AROUND = 2
FOLLOW_RIGHT = 3
STOP = 4


# Helper functions

def detect_light(light_sensors):
    """Return True if light detected strongly on all sides."""
    return (light_sensors[0] > LIGHT_THRESHOLD and
            light_sensors[1] > LIGHT_THRESHOLD and
            light_sensors[2] > LIGHT_THRESHOLD and
            light_sensors[5] > LIGHT_THRESHOLD and
            light_sensors[6] > LIGHT_THRESHOLD and
            light_sensors[7] > LIGHT_THRESHOLD)


# Main controller

robot = Robot()

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Distance sensors
ds_names = ["ds0","ds1","ds2","ds3","ds4","ds5","ds6","ds7"]
ds = [robot.getDevice(name) for name in ds_names]
for sensor in ds:
    sensor.enable(TIME_STEP)

# Light sensors
ls_names = ["ls0","ls1","ls2","ls3","ls4","ls5","ls6","ls7"]
ls = [robot.getDevice(name) for name in ls_names]
for sensor in ls:
    sensor.enable(TIME_STEP)

state = FOLLOW_LEFT
turn_counter = 0  # used for 180° turn timing


# Control Loop

while robot.step(TIME_STEP) != -1:

    # Read sensors
    dist = [s.getValue() for s in ds]
    light = [s.getValue() for s in ls]

    front = dist[0]  # front sensor
    left_side = dist[5]  # left wall sensor
    right_side = dist[2]  # right wall sensor


    # State Machine


    # FOLLOW LEFT 
    if state == FOLLOW_LEFT:
        if detect_light(light):
            state = TURN_AROUND
            turn_counter = 0
            continue

        if front > FRONT_OBSTACLE_THRESHOLD:
            state = AVOID_OBSTACLE
            continue

        # Wall following logic (left side)
        if left_side < WALL_DISTANCE_THRESHOLD:
            # Too close → steer right
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED * 0.5)
        elif left_side > WALL_DISTANCE_THRESHOLD:
            # Too far → steer left
            left_motor.setVelocity(MAX_SPEED * 0.5)
            right_motor.setVelocity(MAX_SPEED)
        else:
            # Good distance → go straight
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)

    #  AVOID OBSTACLE 
    elif state == AVOID_OBSTACLE:
        # Turn right until front is clear
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(-MAX_SPEED * 0.5)

        if front < FRONT_OBSTACLE_THRESHOLD:
            state = FOLLOW_LEFT

    #  TURN AROUND (180°) 
    elif state == TURN_AROUND:
        # Hardcoded turn duration (tune this!)
        turn_counter += 1
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(-MAX_SPEED)

        if turn_counter > 40:  # ~180° turn
            state = FOLLOW_RIGHT

    #  FOLLOW RIGHT 
    elif state == FOLLOW_RIGHT:
        if detect_light(light):
            state = STOP
            continue

        if front > FRONT_OBSTACLE_THRESHOLD:
            state = AVOID_OBSTACLE
            continue

        # Wall following logic (right side)
        if right_side < WALL_DISTANCE_THRESHOLD:
            # Too close → steer left
            left_motor.setVelocity(MAX_SPEED * 0.5)
            right_motor.setVelocity(MAX_SPEED)
        elif right_side > WALL_DISTANCE_THRESHOLD:
            # Too far → steer right
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED * 0.5)
        else:
            left_motor.setVelocity(MAX_SPEED)
            right_motor.setVelocity(MAX_SPEED)

    #  STOP 
    elif state == STOP:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
