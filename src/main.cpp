#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <numbers>
#define as(a, b, c, d) for (auto a = b; a < c; a += d)
#define de(a, b, c, d) for (auto a = b; a > c; a -= d)

int sortedColor = 0; //0 = keep blue, 1 = keep red, 2 = none
bool autonColor; bool autonSide;//T = blue, F = red; T = close, F = far
const int wheelCirc = 220; // in mm
const int driveEncoders = 300; // ticks per revolution
const double trackWidth = 10.8 * 25.4; // conversion to mm
int lbStates[3] = {0,100,200}; //list of all the states
int lbState = 0; //current state it is in
const int lbTotalStates = sizeof(lbStates) / sizeof(lbStates[0]); //total number of states

pros::MotorGroup left ({1, 2, 3}, pros::MotorGearset::blue);
pros::MotorGroup right({4, 5, 6}, pros::MotorGearset::blue);
pros::Motor roller (7,pros::MotorGearset::green);// i defined thesge for you guys according to discord but follow the rest according to shyam (P.S. move(127))
pros::Motor chain (-8,pros::MotorGearset::blue);
pros::Motor lb (9,pros::MotorGearset::blue);
pros::Rotation lbRotation (10);
pros::Controller ctrl (CONTROLLER_MASTER); //controller here
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_left_button(){
  sortedColor++;
    if (sortedColor > 2) {
      sortedColor = 0;
    }
}

void on_center_button() {
  autonColor = !autonColor;
}

void on_right_button() {
  autonSide = !autonSide;
}

void ring_detected(){}

void ring_not_detected(){}

//Moves the robot forward and backward
void drive(int inchesDist, bool forward, int rpm) {
  left.tare_position_all(); right.tare_position_all();
  double mmDist = inchesDist * 25.4;
  double rotations = round(10*(mmDist / wheelCirc)) * 0.1;
  double ticks = round(rotations * driveEncoders);
  double pause = (rotations / rpm) * 60000;

  if (forward) {
    left.move_absolute(ticks, rpm);
    right.move_absolute(ticks, rpm);
  } else {
    left.move_absolute(-ticks, rpm);
    right.move_absolute(-ticks, rpm);
  }

  pros::delay(pause + 100);
}

void turn(double degrees, bool turnLeft, int rpm){
  left.tare_position_all();right.tare_position_all();
  double turnCirc = std::numbers::pi * trackWidth;
  double arcLen = (degrees / 360) * turnCirc;
  double rotations = arcLen / wheelCirc;
  double ticks = round(rotations * driveEncoders);
  double pause = (rotations / rpm) * 60 * 1000;

  if (turnLeft) {
    left.move_absolute(-ticks, rpm);
    right.move_absolute(ticks, rpm);  
  } else {
    left.move_absolute(ticks, rpm);
    right.move_absolute(-ticks, rpm);
  }

  pros::delay(pause + 100);
}
void ladyBrownCycle(bool forward){
  if(forward){
    lbState++;
  }
  else{
    lbState--;
  }
  lbState = lbState % lbTotalStates;
}
void ladyBrownSet(){
  int lbsense = 1.5;
  int error = (lbStates[lbState] - lbRotation.get_position());
  int movePower = lbsense * error;
  lb.move(movePower);
}
// also add drive functions for auton since you got drivetrain done to get things more expeditied
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();// Sets up LLEMU (https://pros.cs.purdue.edu/v5/tutorials/topical/llemu.html)
  pros::lcd::register_btn0_cb(on_left_button); pros::lcd::register_btn1_cb(on_center_button); pros::lcd::register_btn2_cb(on_right_button);
  pros::lcd::print(0, "Hi"); pros::lcd::print(5, "Initialized");
  pros::Task([]{
    if(sortedColor == 0){//auton color info
      pros::lcd::print(1, "LB: Sorting for BLUE"); ctrl.print(1, 0, "LB: Sorting for BLUE");
    } else if (sortedColor == 1) {
      pros::lcd::print(1, "LB: Sorting for RED"); ctrl.print(1, 0, "LB: Sorting for RED");
    } else {
      pros::lcd::print(1, "LB: Sorting for N/A"); ctrl.print(1, 0, "LB: Sorting for N/A");
    }

    if(autonColor) {
      pros::lcd::print(2, "CB: BLUE side auton");
    } else {
      pros::lcd::print(2, "CB: RED side auton");
    }

    if(autonSide) {
      pros::lcd::print(3, "RB: CLOSE side auton");
    } else {
      pros::lcd::print(3, "RB: FAR side auton");
    }
    //insert temperature flags when all the motors are defined
  });
  pros::Task([]{
    ladyBrownSet(); //rotates the lady brown thing to the state
  });
  /*It's good to have an lcd layout to give flags etc to the driver; you can do this through pros::lcd::print() which is to the brain or
  ctrl.print() which is to the controller, it's up to you to decide where!
  (example from mentor code):
  lcd layout (max 8 lines):
  0: hi (can be changed/removed later)
  1: left button setting - color sort fling
  2: mid button setting - auton color     <-- maybe do these toggles for later?
  3: right button setting - auton side    <--
  4: temp flags - overheat or not
  5: comp ctrl mode flag - what mode it is in right now
  */
	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {pros::lcd::print(5, "Disabled");}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {pros::lcd::print(5, "Competition Initialize");}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {pros::lcd::print(5, "Autonomous");}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

  pros::lcd::print(5, "OpControl");
  while (true) {
		//temp flags
    float dtLeftOT = ((round(10.0*((left.get_temperature(0) + left.get_temperature(1) + left.get_temperature(2))/3.0)))/10.0);
    float dtRightOT = ((round(10.0*((right.get_temperature(0) + right.get_temperature(1) + right.get_temperature(2))/3.0)))/10.0);
    //float chainOT = chain.get_temperature();
    //float lbOT = lb.get_temperature();
    //float mogoOT = mogo.get_temperature();
    //printing the overtemp flags on to lcd
    //pros::lcd::print(4, "DTL%.1f DTR%.1f Chain%.1f LB%.1f Mogo%.1f", dtLeftOT, dtRightOT, chainOT, lbOT, mogoOT);
		// Arcade control scheme
    int power = ctrl.get_analog(ANALOG_LEFT_Y);
    int turn;
    if(ctrl.get_digital(DIGITAL_Y)) {// modifier
      turn = (ctrl.get_analog(ANALOG_RIGHT_X)) / 2;
    }
    else {
      turn = ctrl.get_analog(ANALOG_RIGHT_X);
    }
    int powerL = power + turn;
    int powerR = power - turn;
      
    //dt
    left.move(powerL);
    right.move(powerR);

    if(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      roller.move(127);
    } if(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      roller.move(-128);
    } if(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      chain.move(127);
    } if(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      chain.move(-128);
    }

    if(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
      ladyBrownCycle(true);
    }
    if(ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
      ladyBrownCycle(false);
    }
		pros::delay(20);                               // Run for 20 ms then update
	}
}