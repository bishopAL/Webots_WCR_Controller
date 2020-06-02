"""gecko_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
print('Simulation Starts!')
# create the Robot instance.
robot = Robot()
MOTOR_NAME = ['RF0', 'RF1', 'RF2',
                'RH0', 'RH1', 'RH2',
                'LF0', 'LF1', 'LF2',
                'LH0', 'LH1', 'LH2',]
MOTOR_HANDLE_LIST = []
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
for item in MOTOR_NAME:
    MOTOR_HANDLE_LIST.append(robot.getMotor(item))

for item in MOTOR_HANDLE_LIST:
    item.setPosition(0.0)
    print('done')

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
