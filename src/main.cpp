#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep odom color sort rorational on lb
#include "pros/misc.h"
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor intake(2);
pros::Motor stage2(-16);
pros::Motor outtake(-12);

pros::ADIDigitalOut MatchL('F');
pros::ADIDigitalOut Descore('G');

pros::Imu imu1(14);

// pros::ADIDigitalOut color_sort('E');
// pros::Rotation ArmR(21);
// pros::Imu imu(11);
// pros::Optical color(8); 
// pros::Distance distance(1);

// pros::Rotation HOdom(13);
// lemlib::TrackingWheel HTracking(&HOdom, lemlib::Omniwheel::NEW_2, -2);


// motor groups
pros::MotorGroup leftMotors({-4, -7, -9},
                            pros::MotorGearset::blue); // left motor group - all reversed
pros::MotorGroup rightMotors({3, 5, 8}, pros::MotorGearset::blue); // right motor group - All same direction


// // TRACKING WHEELS:
// // horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
// pros::Rotation horizontalEnc(20);
// // vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(19);
// // horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// // vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              15, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 2.75" omnis
                              600, // drivetrain rpm is 450
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            //30
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup 3
                                            2, // small error range, in inches 1
                                            200, // small error range timeout, in milliseconds 100
                                            4, // large error range, in inches 3
                                            600, // large error range timeout, in milliseconds 500
                                            15 // maximum acceleration (slew) 20
);

// angular motion controller
lemlib::ControllerSettings angularController(1.7, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             13, // derivative gain (kD)
                                             0, // anti windup
                                             30, // small error range, in degrees
                                             200, // small error range timeout, in milliseconds
                                             60, // large error range, in degrees
                                             600, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu1 // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
    MatchL.set_value(false);
    Descore.set_value(false);
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

void red_right() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(500);
    MatchL.set_value(false);
    Descore.set_value(false);

    chassis.moveToPoint(19, 38, 4000, {.forwards=true, .maxSpeed=80, .minSpeed=30}, true);
    pros::delay(200);
    intake.move_voltage(12000);
    stage2.move_voltage(4500);
    outtake.move_voltage(2000);
    chassis.waitUntilDone();
    pros::delay(500);
    intake.brake();
    stage2.brake();
    outtake.brake();
    chassis.turnToHeading(140, 3000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed=80, .minSpeed=60});
    chassis.moveToPoint(63, 13, 4000, {.forwards=true, .maxSpeed=80, .minSpeed=60}, false);
    chassis.turnToHeading(180, 3000, {.direction=lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed=80, .minSpeed=60});
    chassis.moveToPoint(68, 35, 4000, {.forwards=false, .maxSpeed=80, .minSpeed=60}, false);
    pros::delay(400);
    intake.move_voltage(12000);
    stage2.move_voltage(12000);
    outtake.move_voltage(12000);
    pros::delay(1000);
    intake.move_voltage(-12000);
    stage2.move_voltage(-12000);
    outtake.move_voltage(-12000);
    pros::delay(400);
    intake.move_voltage(12000);
    stage2.move_voltage(12000);
    outtake.move_voltage(12000);
    pros::delay(1800);
    intake.brake();
    stage2.brake();
    outtake.brake();

    MatchL.set_value(true);
    chassis.moveToPoint(69, 5, 4000, {.forwards=true, .maxSpeed=80, .minSpeed=60}, false);
    chassis.turnToHeading(180, 3000, {.maxSpeed=80, .minSpeed=60});
    intake.move_voltage(12000);
    stage2.move_voltage(12000);
    outtake.move_voltage(12000);
    chassis.moveToPoint(69, -70, 1200, {.forwards=true, .maxSpeed=60, .minSpeed=20}, false);
    intake.brake();
    stage2.brake();
    outtake.brake();

    chassis.moveToPoint(68, 55, 2000, {.forwards=false, .maxSpeed=80, .minSpeed=60}, false);
    intake.move_voltage(12000);
    stage2.move_voltage(12000);
    outtake.move_voltage(12000);
    MatchL.set_value(false);
    pros::delay(1000);
    intake.move_voltage(-12000);
    stage2.move_voltage(-12000);
    outtake.move_voltage(-12000);
    pros::delay(400);
    intake.move_voltage(12000);
    stage2.move_voltage(12000);
    outtake.move_voltage(12000);
    pros::delay(1500);
    intake.brake();
    stage2.brake();
    outtake.brake();
}

void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 10,  0, 1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}
/**
 * Runs in driver control 
 */

void opcontrol() {
    bool MatchL_state = 1;
    bool reset_MatchL = false;
    bool Descore_state = 1;
    bool reset_Descore = false;
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	while (true){
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);


        chassis.tank(leftY, rightY);

        // intake + outtake
        int intake_speed = 0;
        int outtake_speed = 0;
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake_speed = 1;
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_speed = -1;
            outtake_speed = -1;
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake_speed = 1;
            outtake_speed = 1;
        }

        intake.move_voltage(intake_speed * 12000);
        outtake.move_voltage(outtake_speed * 12000);
        
        // MatchL control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			if (reset_MatchL){
				if (MatchL_state == 0){
					MatchL.set_value(false);
				}
				else{
					MatchL.set_value(true);
				}
				MatchL_state = 1 - MatchL_state;
				reset_MatchL = false;
			}
		}
		else {
			reset_MatchL = true;
		}

        // Descore control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			if (reset_Descore){
				if (Descore_state == 0){
					Descore.set_value(false);
				}
				else{
					Descore.set_value(true);
				}
				Descore_state = 1 - Descore_state;
				reset_Descore = false;
			}
		}
		else {
			reset_Descore = true;
		}
	}
}