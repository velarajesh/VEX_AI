#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT2, ratio18_1, false);
motor leftMotorC = motor(PORT3, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT8, ratio18_1, true);
motor rightMotorB = motor(PORT9, ratio18_1, true);
motor rightMotorC = motor(PORT10, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
controller Controller1 = controller(primary);



//TEST DRIVE FUNCTIONS 
bool RemoteControlCodeEnabled = true;

bool DrivetrainNeedsToBeStopped_Controller1 = true;

int rc_auto_loop_function_Controller1() {

  while(true) {
    if(RemoteControlCodeEnabled) {

      int drivetrainLeftSideSpeed = Controller1.Axis2.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position() - Controller1.Axis1.position();
      
      if (abs(drivetrainLeftSideSpeed) < 5 && abs(drivetrainRightSideSpeed) < 5) {
        if (DrivetrainNeedsToBeStopped_Controller1) {
          LeftDriveSmart.stop();
          RightDriveSmart.stop();
          DrivetrainNeedsToBeStopped_Controller1 = false;
        }
      } else {
        DrivetrainNeedsToBeStopped_Controller1 = true;
      }
      
      if (DrivetrainNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      if (DrivetrainNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }

    
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}