#include "main.h"
#include "pros/motors.hpp"
#define as(a, b, c, d) for (auto a = b; a < c; a += d)
#define de(a, b, c, d) for (auto a = b; a > c; a -= d)

pros::MotorGroup left ({1, 2, 3}, pros::MotorGearset::blue);
pros::MotorGroup right({-4, -5, -6}, pros::MotorGearset::blue);
pros::Motor roller (7,pros::MotorGearset::green);// i defined these for you guys according to discord but follow the rest according to shyam (P.S. move_velocity(127))
pros::Motor chain (-8,pros::MotorGearset::blue);
pros::Controller ctrl (CONTROLLER_MASTER); //controller here
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
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
	pros::lcd::set_text(1, "Hello PROS User!");
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
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {}

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


  while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
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
    if(ctrl.get_digital(DIGITAL_Y)) {
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
		pros::delay(20);                               // Run for 20 ms then update
	}
}