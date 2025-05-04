#include <string>
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///

// Get shortest angular difference in degrees (-180 to 180)
double Shortest_angular_error(double current, double target) {
  double error = fmod((target - current + 540.0), 360.0) - 180.0;
  return error;
}

const double setval=37;

bool dwLbs = true;

void LbSet(double targetDeg){
  const double kP =-.6;
  const double tolerance = 2;
  double error = Shortest_angular_error(Rotation.get_position(), setval);
  dwLbs = false;
  pros::lcd::clear_line(0);
  pros::lcd::clear_line(1);
  pros::lcd::clear_line(2);
  pros::lcd::clear_line(3);
  if (error<tolerance) {
    double current_pos = Rotation.get_position();
    error = Shortest_angular_error(current_pos/100, targetDeg);
  
    double speed = error * -kP*6;
      
      // Clamp motor speed
    if (speed > 127) speed = 127;
    if (speed < -127) speed = -127;
 

    LB.move(speed);
 
    std::string CURPOS = std::to_string(current_pos/100);
    std::string ERrrr = std::to_string(error);
    std::string Speed = std::to_string(speed);
  
    pros::lcd::set_text(0, CURPOS);
    pros::lcd::set_text(1, ERrrr);
    pros::lcd::set_text(2, Speed);
      
    pros::delay(10);
  }
  
}


void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!



void RedRush(){
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  chassis.drive_angle_set(100);
  //////////////Mobile Goa101l 1/////////////
  Intake.move(-120);

  Doinker.extend();
  //backward(98.714, 30);
  chassis.pid_drive_set(36_in,100);
  chassis.pid_wait();


// Path
/*
forward(136.43, 30);
turnTo(270.682, 30);
forward(58.311, 30);
turnTo(330.255, 30);
forward(123.118, 30);
turnTo(195.255, 30);
forward(142.458, 30);
turnTo(81.55, 30);
forward(294.33, 30);
turnTo(0, 30);
*/


}

void AWP(){
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  chassis.drive_angle_set(301);

  LB.move(100);

  pros::delay(800);

  LB.move(-100);  

  chassis.pid_drive_set(-65_cm,120);
  chassis.pid_wait();

  LB.move(0);

  chassis.pid_swing_set(ez::LEFT_SWING, 270, 120,20);
  chassis.pid_wait();

  chassis.pid_drive_set(-15_cm,120);
  chassis.pid_wait();

  Clamp.extend();


  pros::delay(200);

  LB.move(0);

//turnTo(178.479, 30);
  chassis.pid_turn_set(180_deg,120);
  chassis.pid_wait();
// Path
  Intake.move(-100);

  chassis.pid_drive_set(60_cm,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-10_cm,120);
  chassis.pid_wait();

    ///forward(134.16, 30);
  Intake.move(-120);
  //turnTo(332.928, 30);

  chassis.pid_swing_set(ez::LEFT_SWING, 350_deg, 110,0);
  chassis.pid_wait();

  Clamp.retract();

  chassis.pid_drive_set(100_cm,120);
  chassis.pid_wait();

  chassis.pid_drive_set(100_cm,100);
  chassis.pid_wait();

  pros::delay(200);

  Intake.move(0);

  chassis.pid_turn_set(232_deg, 120);
  chassis.pid_wait();

  chassis.pid_drive_set(-60.16_cm,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-20_cm,80);
  chassis.pid_wait();

  Clamp.extend();

  pros::delay(200);

  chassis.pid_turn_set(0_deg,120);
  chassis.pid_wait();
// Path
  Intake.move(-120);

  chassis.pid_drive_set(70_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(345_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-100_cm,120);
  chassis.pid_wait();



}


void BlueNegative(){
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  chassis.drive_angle_set(90);

  //Mogo 1

  chassis.pid_drive_set(-60_cm,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-25_cm,90);
  chassis.pid_wait();

  Clamp.extend();

  pros::delay(250);


  Intake.move(-120);

  chassis.pid_turn_set(0_deg,120);
  chassis.pid_wait();


  Intake.move(-100);

  chassis.pid_drive_set(70_cm,100);
  chassis.pid_wait();


  chassis.pid_drive_set(-20_cm,90);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(29_cm,120);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-27_cm,90);
  chassis.pid_wait();

  chassis.pid_turn_set(290_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(35_cm,120);
  chassis.pid_wait();

  pros::delay(500);
  chassis.pid_drive_set(-35_cm,90);
  chassis.pid_wait();  

  chassis.pid_turn_set(180_deg,120, ez::cw);
  chassis.pid_wait();

  chassis.pid_drive_set(70_cm,80);
  chassis.pid_wait();   

  chassis.pid_drive_set(15_cm,40);
  chassis.pid_wait();   
  /*

  //Rings

  chassis.pid_turn_set(315,120);
  chassis.pid_wait();

  

  chassis.pid_drive_set(57_cm,120);
  chassis.pid_wait();

  pros::delay(150);

  chassis.pid_drive_set(-9_cm,120);
  chassis.pid_wait();

  pros::delay(750);

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 120, 20, ez::cw);
  chassis.pid_wait();

  chassis.pid_drive_set(10_cm,120);
  chassis.pid_wait();

  pros::delay(999999);

  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 100, 16, ez::cw);
  chassis.pid_wait();

  chassis.pid_drive_set(30_cm,120);
  chassis.pid_wait();

  pros::delay(99999);
  //Corner

  chassis.pid_turn_set(105,120);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 120, 70);
  chassis.pid_wait();

  chassis.pid_drive_set(50_cm,120);
  chassis.pid_wait();

  //Touch

  pros::delay(500);

  chassis.pid_turn_set(225,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-230_cm,120);
  chassis.pid_wait();

  */
}

void BluePositiveSafe(){
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  chassis.drive_angle_set(239);

  LB.move(100);

  pros::delay(800);

  LB.move(-100);  

  chassis.pid_drive_set(-65_cm,120);
  chassis.pid_wait();

  LB.move(0);

  chassis.pid_swing_set(ez::RIGHT_SWING, 270, 120,20);
  chassis.pid_wait();

  chassis.pid_drive_set(-15_cm,120);
  chassis.pid_wait();

  Clamp.extend();

  ///Mogo 1


  pros::delay(500);

  Intake.move(-100);

  chassis.pid_turn_set(0_deg,120);
  chassis.pid_wait();


  chassis.pid_drive_set(70_cm,120);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-30_cm,120);
  chassis.pid_wait();  

  chassis.pid_turn_set(335_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-70_cm,120);
  chassis.pid_wait();  



  pros::delay(999999);




}

void RedNegative(){
  
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  chassis.drive_angle_set(90);

  //Mogo 1

  chassis.pid_drive_set(-60_cm,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-25_cm,90);
  chassis.pid_wait();

  Clamp.extend();

  pros::delay(250);


  Intake.move(-120);

  chassis.pid_turn_set(180_deg,120);
  chassis.pid_wait();


  Intake.move(-100);

  chassis.pid_drive_set(70_cm,100);
  chassis.pid_wait();

  

  chassis.pid_drive_set(-0_cm,90);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(29_cm,120);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-27_cm,90);
  chassis.pid_wait();

  chassis.pid_turn_set(250_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(39_cm,120);
  chassis.pid_wait();

  pros::delay(1000);
  chassis.pid_drive_set(-39_cm,90);
  chassis.pid_wait();  

  chassis.pid_turn_set(0_deg,120, ez::ccw);
  chassis.pid_wait();

  chassis.pid_drive_set(70_cm,80);
  chassis.pid_wait();   

  chassis.pid_drive_set(15_cm,40);
  chassis.pid_wait();  
  
  ///////Only for Flipping////////////////Start
  //chassis.odom_theta_flip();
  //BlueNegative();
  //////////////////////////////////////End
}

void RedPositiveSafe(){

  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  chassis.drive_angle_set(301);

  LB.move(100);

  pros::delay(800);

  LB.move(-100);  

  chassis.pid_drive_set(-65_cm,120);
  chassis.pid_wait();

  LB.move(0);

  chassis.pid_swing_set(ez::LEFT_SWING, 270, 120,20);
  chassis.pid_wait();

  chassis.pid_drive_set(-15_cm,120);
  chassis.pid_wait();

  Clamp.extend();

  ///Mogo 1


  pros::delay(500);

  Intake.move(-100);

  chassis.pid_turn_set(180_deg,120);
  chassis.pid_wait();


  chassis.pid_drive_set(70_cm,120);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-30_cm,120);
  chassis.pid_wait();  

  chassis.pid_turn_set(205_deg,120);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-70_cm,120);
  chassis.pid_wait();  

}

void PS1(){
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);


  chassis.drive_angle_set(90_deg);

  ////MOGO 1
  LB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  //chassis.pid_drive_set(2.7_in,40);
  //chassis.pid_wait();

  //pros::delay(250);


  Intake.move(-120);

  pros::delay(130);

  Intake.move(0);

  pros::delay(400);

  chassis.pid_drive_set(33_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-40_cm,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-25_cm,90);
  chassis.pid_wait();


  Clamp.extend();

  pros::delay(350);

  Intake.move(-100);

  //Ring 1
  chassis.pid_turn_set(90_deg,120);
  chassis.pid_wait();
  
  Intake.move(-100);
  

  chassis.pid_drive_set(70_cm,120);
  chassis.pid_wait();

  pros::delay(300);

  chassis.pid_drive_set(-10_cm,100);
  chassis.pid_wait();


  //Ring 2 (for LB)

  chassis.pid_turn_set(52_deg,120);
  chassis.pid_wait();


  chassis.pid_drive_set(95_cm,120);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, 120);
  chassis.pid_wait();
  
  //LB set
  pros::delay(200);

  LB.move(90);

  pros::delay(320);

  LB.move(0);

  pros::delay(200);

  //LB Set End///////////////////
  chassis.pid_drive_set(40_cm,120);
  chassis.pid_wait();

  pros::delay(500);


  //Go to RING 3 and score wall

  chassis.pid_drive_set(-45_cm,120);
  chassis.pid_wait();


  Intake.move(-100);

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 120, 0, ez::ccw);
  chassis.pid_wait();


  Intake.move(30);

  pros::delay(200);

  LB.move(80);

  pros::delay(300);

  LB.move(0);

  Intake.move(-100);

  chassis.pid_drive_set(50_cm,80);
  chassis.pid_wait();

  LB.move(70);

  pros::delay(600);

  LB.move(0);

  chassis.pid_drive_set(-35_cm,80);
  chassis.pid_wait();

  LB.move(-100);



  //ring 4,5,6
  chassis.pid_turn_set(270_deg,120);
  chassis.pid_wait();

  LB.move(00);

  chassis.pid_drive_set(60_cm,90);
  chassis.pid_wait();

  chassis.pid_turn_set(265_deg,120);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_set(60_cm,90);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg,120);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_set(30_cm,90);
  chassis.pid_wait();

  pros::delay(500);
  // Ring 6

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, 120, 20, ez::cw);
  chassis.pid_wait();
  
  chassis.pid_drive_set(52_cm,120);
  chassis.pid_wait();

  pros::delay(400);


  //Corner

  
  chassis.pid_drive_set(-30_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(135_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-40_cm,120);
  chassis.pid_wait();

  Clamp.retract();


  pros::delay(250);

  
  Intake.move(60);

  //Get in Position for 2

  chassis.pid_drive_set(30_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg,120);
  chassis.pid_wait();  

  chassis.pid_drive_set(-10_cm,80);
  chassis.pid_wait();


  

}

void PS2(){
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);

  chassis.drive_angle_set(0_deg);

  chassis.pid_drive_set(-55_in,120);
  chassis.pid_wait();

  pros::delay(300);

  chassis.pid_drive_set(-10_in,80);
  chassis.pid_wait();

  chassis.pid_drive_set(-10_in,60);
  chassis.pid_wait();

  Clamp.extend();

  pros::delay(600);

  chassis.pid_drive_set(4_in,80);
  chassis.pid_wait();

  //Ring 1
  chassis.pid_turn_set(90_deg,120);
  chassis.pid_wait();
  
  Intake.move(-100);
  

  chassis.pid_drive_set(70_cm,120);
  chassis.pid_wait();

  pros::delay(300);

  chassis.pid_drive_set(-10_cm,120);
  chassis.pid_wait();

  //Ring 2 (for LB)

  chassis.pid_turn_set(135_deg,120);
  chassis.pid_wait();


  chassis.pid_drive_set(70_cm,120);
  chassis.pid_wait();


  /*////NextState();
  for (int i = 0; i < 50; i++) {
    LbSet(setval);
    pros::delay(10);
  }

  //LB set
  //LB.move_absolute(300, 80);

  pros::delay(500);
  
  //LB Set End///////////////////

  chassis.pid_drive_set(60_cm,80);
  chassis.pid_wait();

  chassis.pid_drive_set(10_cm,100);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-10_cm,100);
  chassis.pid_wait();




  //Go to RING 3 and score wall

  chassis.pid_drive_set(-27_cm,120);
  chassis.pid_wait();

  for (int i = 0; i < 12; i++) {
    Intake.move(15);

    pros::delay(20);

    Intake.move(-100);

    pros::delay(20);
  }


  

  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 120, 0, ez::cw);
  chassis.pid_wait();


  Intake.move(15);

  pros::delay(50);

  //LB up
  LB.move_absolute(400, 80);

  pros::delay(500);

  chassis.pid_drive_set(10_cm,80);
  chassis.pid_wait();

  Intake.move(-100);

  chassis.pid_drive_set(40_cm,40);
  chassis.pid_wait();

  //LBscore

  LB.move_absolute(1500, 80);

  pros::delay(1000);

  //reset

  LB.move_absolute(0, 100);

  chassis.pid_drive_set(-35_cm,50);
  chassis.pid_wait();

  //pros::delay(999999);////////////////////////////////////////////////////////////

*/
  ////////////////////////////repcode///////////////


  chassis.pid_turn_set(180_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(30_cm,120);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_set(-25_cm,120);
  chassis.pid_wait();

  /////////////////////////////////////////////
  //ring 4,5,6

  chassis.pid_turn_set(270_deg,120);
  chassis.pid_wait();

  LB.move(00);

  chassis.pid_drive_set(66_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg,120);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_set(60_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(260_deg,120);
  chassis.pid_wait();

  pros::delay(400);

  chassis.pid_drive_set(15_cm,120);
  chassis.pid_wait();

  pros::delay(500);



  // Ring 6

  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 120, 20, ez::ccw);
  chassis.pid_wait();

  chassis.pid_drive_set(50_cm,120);
  chassis.pid_wait();

  pros::delay(250);

  //Corner

  
  chassis.pid_drive_set(-30_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(-30_cm,120);
  chassis.pid_wait();

  Clamp.retract();

  pros::delay(250);

  Intake.move(0);

  //Get in Position for PS3
  chassis.pid_drive_set(50_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg,120);
  chassis.pid_wait();

  chassis.pid_drive_set(120_cm,120);
  chassis.pid_wait();

  
  chassis.pid_turn_set(250_deg,120);
  chassis.pid_wait();


  chassis.pid_drive_set(-84_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(230_deg,120);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-90_cm,120);
  chassis.pid_wait();
  




}

void PS3(){



  //2 Rings
  
  chassis.pid_turn_set(5_deg,120);
  chassis.pid_wait();

  Doinker.extend();

  chassis.pid_drive_set(150_cm,120);
  chassis.pid_wait();


  //3rd ring

  chassis.pid_drive_set(-150_cm,120);
  chassis.pid_wait();

  chassis.pid_turn_set(355_deg,120);
  chassis.pid_wait();

  Doinker.extend();

  chassis.pid_drive_set(-150_cm,120);
  chassis.pid_wait();






}

void PS4(){
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);


  chassis.drive_angle_set(90_deg);

  ////MOGO 1
  LB.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  //LB set
  LB.move_absolute(350, 80);

  pros::delay(500);
  
  //LB Set End///////////////////

  Intake.move(-100);

  chassis.pid_drive_set(50_cm,120);
  chassis.pid_wait();

  pros::delay(500);

  Intake.move(15);

  pros::delay(100);

  Intake.move(-100);

  pros::delay(100);

  Intake.move(15);

  pros::delay(100);

  LB.move_absolute(700 , 80);

  pros::delay(250);

  Intake.move(-100);

  chassis.pid_drive_set(50_cm,120);
  chassis.pid_wait();










}

void programmingSkills(){
  //PS1();
  PS2();
  PS3();
  //PS4();
}
// . . .