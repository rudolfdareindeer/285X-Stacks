#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
pros::MotorGroup left_motors({ -19, 18,-17}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({13, -11, 12}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6   
pros::Motor conveyor(14, pros::MotorGearset::green);        
pros::Motor fsintake(-20, pros::MotorGearset::green);     
pros::Motor lift(-21, pros::MotorGearset::red);
pros::ADIPort clamp('A', pros::E_ADI_DIGITAL_OUT);   
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/**  Track Width:
*	 Wheel Diam: 
*	 Horizontal Drift: 2
*/
pros::Imu imu(16);	        
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.5, // 12.5 inch track width
                              lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
                              400, // drivetrain rpm is 400
                              2 // horizontal drift is 2 
);
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
// angular PID controller
lemlib::ControllerSettings angular_controller(2  , // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
    chassis.calibrate();
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
 * for non-competi  tion testing purposes.        
 *  
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the rob ot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() { 
    
    int ally = 1000;
  //  chassis.setPose(58.5,8.165,315);
    //chassis.moveToPose(62.6, 4.97, 315, 2000,   {.forwards=false});
    

    chassis.setPose(58,12,0);
    chassis.moveToPose(58, 0, 0, 2000, {.forwards=false});
    chassis.turnToHeading(270, 1000);
    pros::delay(1000);
    right_motors.move(-30);
    left_motors.move(-30);
    pros::delay(300);
    right_motors.move(0);
    left_motors.move(0);
    
    conveyor.move(127);
    pros::delay(2000);
    right_motors.move(30);
    left_motors.move(30);
    pros::delay(300);
    right_motors.move(0);
    left_motors.move(0);
    
    chassis.turnToPoint(33, 23, 1000, {.forwards=false});  
    //chassis.moveToPoint(33, 23, 3000,{.forwards=false});
    pros::delay(1001);
    right_motors.move(-50);
    left_motors.move(-50);
    pros::delay(1500);
    clamp.set_value(true);
    pros::delay(300);
    right_motors.move(0);
    left_motors.move(0);
    pros::delay(1000);
    chassis.turnToPoint(24, 41, 2000);
    fsintake.move(127);
    chassis.moveToPoint(24, 41, 3000);
    chassis.turnToPoint(65, 66.4, 2000);
    chassis.moveToPoint(65, 66.4, 3000);
    chassis.turnToPoint(28.2, 7.2, 2000);
    chassis.moveToPoint(28.2, 7.2, 3000);

   // chassis.setPose(0,0,0);
    //chassis.moveToPose(0,24, 0, 1000);





    //start with clamp towards ring, drive forward, turn clamp towards ally stake, drive


    
    
   // chassis.turnToHeading(90, 200000);
}   

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
 void driveTask(void* param) {
    while (true) {
        // Logic for opening/closing clamp or holding mobile goal
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with arcade drive
        chassis.arcade(leftY, rightX, false, 0.5);
        // move the chassis with tank drive
        //chassis.tank(leftY, rightY);
        pros::delay(20);
    }
   
 }
 void conveyorTask(void* param) {
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            conveyor.move(127);
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            conveyor.move(-127);
    } else{
            conveyor.move(0);
        }
        pros::delay(20);
    }
 }
 
 
 void intakeTask(void* param) {
    while (true) {
          if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            fsintake.move(127);
        } else {
            fsintake.move(0);
        }
    }
 }
 void liftTask(void* param) {
    while (true) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            lift.move(127);      // Raise lift
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            lift.move(-127);     // Lower lift
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            lift.move_velocity(0);        
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            lift.move_velocity(0); 
        } else {
            lift.move_velocity(0);
        }   
        pros::delay(20);
    }
 }
 void clampTask(void* param) {
    bool clamp_engaged = false;
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            clamp_engaged = !clamp_engaged;  // Toggle clamp
            clamp.set_value(clamp_engaged);
            pros::delay(200);               // Prevent rapid toggling
        }

        pros::delay(20);
    }
}
void opcontrol() {
    pros::Task drive(driveTask, NULL, "Drive Task");
    pros::Task conveyor(conveyorTask, NULL, "Conveyor Task");
    pros::Task intake(intakeTask, NULL, "Intake Task");
    pros::Task lift(liftTask, NULL, "Lift Task");
    pros::Task clamp(clampTask, NULL, "Clamp Task");
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
                //int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
              //int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        // move the chassis with arcade drive
        // move the chassis with tank drive
             //  chassis.tank(leftY, rightY);
        // spin conveyor on L2 hold
          /*if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            conveyor.move(127);
        } else{
            conveyor.move(0);
        }

        //spin first stage intake with R2 hold
          if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            fsintake.move(127);
        } else {
            fsintake.move(0);
        }



        // delay to save resources
        pros::delay(25);
        */
    }
}   