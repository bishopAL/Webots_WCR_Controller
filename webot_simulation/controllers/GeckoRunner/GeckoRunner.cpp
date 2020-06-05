// File:          GeckoRunner.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
// All the webots classes are defined in the "webots" namespace
using namespace webots;
const char* MOTION_NAMES[4][3] = {
   {"RF0", "RF1", "RF2"},
   {"RH0", "RH1", "RH2"},
   {"LF0", "LF1", "LF2"},
   {"LH0", "LH1", "LH2"}
};
const char* SENSOR_NAMES[4][3] = {
   {"RF0 sensor", "RF1 sensor", "RF2 sensor"},
   {"RH0 sensor", "RH1 sensor", "RH2 sensor"},
   {"LF0 sensor", "LF1 sensor", "LF2 sensor"},
   {"LH0 sensor", "LH1 sensor", "LH2 sensor"}
};
Motor *limb_motors[4][3];
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<3;j++)
    {
      limb_motors[i][j] = robot->getMotor(MOTION_NAMES[i][j]);
    }
  }
  for(int i=0;i<4;i++)
  {
    for(int j=0;j<3;j++)
    {
      limb_motors[i][j]->setPosition(0.0);
    }
  }
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
