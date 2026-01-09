#include "vex.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1 = controller(primary);

// motors (ratio18_1 for standard Green cartridges)(false means spin forward, true means backward)
motor LeftFront = motor(PORT9, ratio18_1, false);
motor LeftBack = motor(PORT1, ratio18_1, false);

motor RightFront = motor(PORT10, ratio18_1, true);
motor RightBack = motor(PORT2, ratio18_1, true);

motor IntakeMotor = motor(PORT6, ratio18_1, false);
motor ConveyorMotor = motor(PORT3, ratio18_1, false);

inertial InertialSensor = inertial(PORT15);
optical OpticalSensor = optical(PORT15);

// motor groups (control entire side at once)
motor_group LeftDrive = motor_group(LeftBack, LeftFront);
motor_group RightDrive = motor_group(RightBack, RightFront);

// drivetrain, telling robot its own dimensions. (LeftGroup, RightGroup, WheelTravel, TrackWidth, 
//     Wheelbase, units, GearRatio)
smartdrive DDrive = smartdrive(LeftDrive, RightDrive, InertialSensor, 319.19, 295, 40, mm, 1);

bool smartIntake() {
    IntakeMotor.spin(forward, 100, percent);
    ConveyorMotor.spin(forward, 100, percent);
    if (OpticalSensor.isNearObject()) { //need to add Optical Sensor to the block
    // Logic for when the Pushback object is grabbed
      IntakeMotor.stop(brake);
      return true;
    }
    return false;
}

void regularIntake(){

}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // hold mode -- most powerful brakes to prevent moving before competition starts
  LeftDrive.setStopping(hold);
  RightDrive.setStopping(hold);
  OpticalSensor.setLightPower(100, percent);
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(100, msec);
  }
  Brain.Screen.print("Novas Ready!");
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                         Autonomous Task - 15 seconds                      */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // get ready for autonomous by exiting hold
  LeftDrive.setStopping(brake);
  RightDrive.setStopping(brake);
  ConveyorMotor.setStopping(hold);
  // set speed (avoid 100% to maintain grip)
  DDrive.setDriveVelocity(60, percent);
  DDrive.setTurnVelocity(40, percent);
  
  // AUTONOMOUS MOVEMENTS
  DDrive.driveFor(forward, 24, inches); 
  DDrive.turnFor(left, 90, degrees);
  //DDrive.driveFor(reverse, 12, inches);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                    User Control Task - 1 min 45 seconds                   */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // get ready for user control
  DDrive.setDriveVelocity(100, percent);
  LeftDrive.setStopping(coast);
  RightDrive.setStopping(coast);
  ConveyorMotor.setStopping(hold); // prevent sliding/slipping

  // User control code here, inside the loop
  while (true) {
    // Drive logic
    int forwardVal = Controller1.Axis3.position();
    int turnVal = Controller1.Axis1.position();

    // 5% deadzone so that joystick being close enough to center counts as 0
    if (forwardVal < 5 && forwardVal > -5) forwardVal = 0;
    if (turnVal < 5 && turnVal > -5) turnVal = 0;

    LeftDrive.spin(vex::forward, forwardVal + turnVal, percent);
    RightDrive.spin(vex::forward, forwardVal - turnVal, percent);

    // intake and conveyor commands
    if (Controller1.ButtonR1.pressing()) {
      // R1: Suck in and move conveyor up
      IntakeMotor.spin(vex::forward, 100, percent);
      ConveyorMotor.spin(vex::forward, 100, percent);
    } 
    else if (Controller1.ButtonR2.pressing()) {
      // R2: Spit out everything
      IntakeMotor.spin(vex::reverse, 100, percent);
      ConveyorMotor.spin(vex::reverse, 100, percent);
    }
    else if (Controller1.ButtonL1.pressing()) {
      // L1: Only move conveyor up (useful for scoring)
      ConveyorMotor.spin(vex::forward, 100, percent);
      IntakeMotor.stop(brake);
    }
    else if (Controller1.ButtonL2.pressing()) {
      // L2: Only move conveyor down
      ConveyorMotor.spin(vex::reverse, 100, percent);
      IntakeMotor.stop(brake);
    }
    else {
      // STOP EVERYTHING
      IntakeMotor.stop(brake);
      ConveyorMotor.stop(hold); 
    }

    /*
    // OPTIONAL: BRAKE TOGGLE ---
    // Press 'Down' to be unpushable, 'Up' to coast and save battery
    // May be too complicated for user...
    if (Controller1.ButtonDown.pressing()) {
        LeftDrive.setStopping(hold);
        RightDrive.setStopping(hold);
    } else if (Controller1.ButtonUp.pressing()) {
        LeftDrive.setStopping(coast);
        RightDrive.setStopping(coast);
    }
    */

    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resource
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
