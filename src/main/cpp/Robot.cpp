/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>
#include <memory>
#include <string>
#include <array>

#include <frc/IterativeRobot.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/Spark.h>
#include <frc/Talon.h>
#include <frc/Encoder.h>
//#include <frc/WPILib.h>
#include <frc/PowerDistributionPanel.h>
#include <cameraServer/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/Servo.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
// #include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
//#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
// #include <C:/Users/Programming/.gradle/caches/transforms-2/files-2.1/bcaf719eab4760b22c0d3083c34a9489/hal-cpp-2020.3.2-headers/mockdata/MockHooks.h>
// #include <C:/Users/Programming/.gradle/caches/transforms-2/files-2.1/bcaf719eab4760b22c0d3083c34a9489/hal-cpp-2020.3.2-headers/mockdata/SimDeviceData.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <frc/RobotDrive.h>

#include "rev/SparkMax.h"
#include <frc/Compressor.h>
#include <frc/Talon.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
// #include <frc/AddressableLED.h>
#include <math.h>

cs::UsbCamera camera0;
cs::UsbCamera camera1;
cs::VideoSink server;
frc::Joystick one{0}, two{1};
//rev::SparkMax intake{4}, outtake{5};
rev::SparkMax top{5}, intake{0}, bottom{8};
frc::Servo pan{6},tilt{7};
int stage = 0;
double xyz[] = {0.0, 0.0, 0.0};
// frc::Talon frontLeft{2}, frontRight{0}, backRight{3}, backLeft{1}, out{8};

ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontLeft = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(2);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontRight = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(1);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *backLeft= new ctre::phoenix::motorcontrol::can::WPI_TalonFX(3);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *backRight = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(0);
ctre::phoenix::motorcontrol::can::TalonSRX *talon = new ctre::phoenix::motorcontrol::can::TalonSRX(10);

frc::RobotDrive myRobot{*frontLeft, *backLeft, *frontRight, *backRight};
frc::Timer timer, shootTimer;


//frc::SendableChooser autoChoice;
frc::Solenoid ballUnstuck{0};
frc::DoubleSolenoid ballIn{3, 7}, ballStorage{2, 1};
frc::Compressor *compressor = new frc::Compressor(0);

ctre::phoenix::sensors::PigeonIMU pigeon{talon};
//0.65 is the ideal sensitivity
double speed = 0.0, turn = 0.0, autoturn = 0.5, sensitivity = 1.0, turnKey, avgDist = 0.0, currentTime = 0.0, prevTime = 0.0, maxTime = 0, maxSpeed = 0, autospeed = 0.65;
bool isUpPressed, isDownPressed;
double sP,tN;
int16_t accel[3];

static constexpr int kLength = 278;

// PWM port 9
// Must be a PWM header, not MXP or DIO
//frc::AddressableLED m_led{0};
// std::array<frc::AddressableLED::LEDData, kLength> m_ledBuffer;  // Reuse the buffer
// // Store what the last hue of the first pixel is
// int firstPixelHue = 0;

// void Rainbow() {
//   // For every pixel
//   for (int i = 0; i < kLength; i++) {
//     // Calculate the hue - hue is easier for rainbows because the color
//     // shape is a circle so only one value needs to process
//     const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
//     // Set the value
//     m_ledBuffer[i].SetHSV(pixelHue, 255, 64);
//   }
//   // Increase by to make the rainbow "move"
//   firstPixelHue += 2;
//   // Check bounds
//   firstPixelHue %= 180;
// }

void resetEncoders() {
  frontLeft->SetSelectedSensorPosition(0);
  backLeft->SetSelectedSensorPosition(0);
  frontRight->SetSelectedSensorPosition(0);
  backRight->SetSelectedSensorPosition(0);
}

int heading() {
  int z = (int)xyz[2];
  if (z > 0) {
    return (z % 360);
  }
  else {
    while (z < 0) {
      z = z + 360;
    }
    return (z % 360);
  }
}

double trueMap(double val, double valHigh, double valLow, double newHigh, double newLow)
{
	double midVal = ((valHigh - valLow) / 2) + valLow;
	double newMidVal = ((newHigh - newLow) / 2) + newLow;
	double ratio = (newHigh - newLow) / (valHigh - valLow);
	return (((val - midVal) * ratio) + newMidVal);
}

void calibratePigeon() {
  pigeon.SetAccumZAngle(0);
  pigeon.SetCompassAngle(0.0);
  pigeon.SetCompassDeclination(0);
  pigeon.SetFusedHeading(0);
  pigeon.SetFusedHeadingToCompass(0);
  pigeon.SetYaw(0);
  pigeon.SetYawToCompass(0);
}

void Robot::RobotInit() {
  camera0 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
  server = frc::CameraServer::GetInstance()->GetServer();
  camera0.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  frc::CameraServer::GetInstance()->StartAutomaticCapture();
  // m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  // m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  // frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Timer", timer.Get());
  timer.Reset();
  timer.Start();
  calibratePigeon();
  // m_led.SetLength(kLength);
  // m_led.SetData(m_ledBuffer);
  // m_led.Start();
  frontLeft->SetSelectedSensorPosition(0.0);
  backLeft->SetSelectedSensorPosition(0.0);
  frontRight->SetSelectedSensorPosition(0.0);
  backRight->SetSelectedSensorPosition(0.0);
}

void Robot::RobotPeriodic() {
  pigeon.GetAccumGyro(xyz);
  frc::SmartDashboard::PutNumber("Heading z: ", heading());
  frc::SmartDashboard::PutNumber("Timer", timer.Get());
  frc::SmartDashboard::PutNumber("FrontLeft Distance: ", (double)frontLeft->GetSelectedSensorPosition()/6612.5);
  frc::SmartDashboard::PutNumber("FrontRight Distance: ", (double)frontRight->GetSelectedSensorPosition()/6612.5);
  frc::SmartDashboard::PutNumber("Backleft Distance: ", (double)backLeft->GetSelectedSensorPosition()/6612.5);
  frc::SmartDashboard::PutNumber("Backright Distance: ", (double)backRight->GetSelectedSensorPosition()/6612.5);
  frc::SmartDashboard::PutNumber("FrontLeft Velocity: ", (double)frontLeft->GetSelectedSensorVelocity()/6612.5);
  frc::SmartDashboard::PutNumber("FrontRight Velocity: ", (double)frontRight->GetSelectedSensorVelocity()/6612.5);
  frc::SmartDashboard::PutNumber("Backleft Velocity: ", (double)backLeft->GetSelectedSensorVelocity()/6612.5);
  frc::SmartDashboard::PutNumber("Backright Velocity: ", (double)backRight->GetSelectedSensorVelocity()/6612.5);
  frc::SmartDashboard::PutNumber("Heading: ",pigeon.GetAbsoluteCompassHeading());
  // Rainbow();
  // m_led.SetData(m_ledBuffer);
}
bool stageDone = false;

void Robot::AutonomousInit() {
  timer.Reset();
  timer.Start();
  shootTimer.Reset();
  // outtake.Set(1.0);
  ballStorage.Set(frc::DoubleSolenoid::Value::kOff);
  calibratePigeon();
  pigeon.SetAccumZAngle(0);
  currentTime = 0;
  prevTime = 0;
  stage = 0;
  resetEncoders();
  stageDone = false;
}



void Robot::AutonomousPeriodic() {
  avgDist = -(-(double)frontLeft->GetSelectedSensorPosition()-(double)backLeft->GetSelectedSensorPosition()+(double)frontRight->GetSelectedSensorPosition()+(double)backRight->GetSelectedSensorPosition())/4.0;
  avgDist /= 6612.5;
  frc::SmartDashboard::PutNumber("Timer", timer.Get());
  frc::SmartDashboard::PutNumber("Average Distance", avgDist);
  frc::SmartDashboard::PutNumber("Stage Time: ", currentTime-prevTime);
  frc::SmartDashboard::PutNumber("Stage: ", stage+1);
  currentTime = timer.Get();
  // if (stage == 0) {
  //   if (avgDist < 15) {myRobot.ArcadeDrive(0.5, 0.0);}
  //   else if (!stageDone) {prevTime = currentTime; resetEncoders(); stageDone = true;}
  //   else {
  //     myRobot.ArcadeDrive(0.0, 0.0);
  //     if (currentTime - prevTime >= 0.5) {stage++; stageDone = false;}
  //   }
  // }
  // if (stage == 01) {
  //   if (heading() < 40 || heading() > 50) {myRobot.ArcadeDrive(0.0, 0.5);}
  //   else if (!stageDone) {prevTime = currentTime; resetEncoders(); stageDone = true;}
  //   else {
  //     myRobot.ArcadeDrive(0.0, 0.0);
  //     if (currentTime - prevTime >= 0.5) {stage++; stageDone = false;}
  //   }
  // }
  // if (stage == 2) {
  //   if (avgDist < 15) {myRobot.ArcadeDrive(0.5, 0.0);}
  //   else if (!stageDone) {prevTime = currentTime; resetEncoders(); stageDone = true;}
  //   else {
  //     myRobot.ArcadeDrive(0.0, 0.0);
  //     if (currentTime - prevTime >= 0.5) {stage++; stageDone = false;}
  //   }
  // }
  // if (stage == 03) {
  //   if (heading() < 40+90 || heading() > 50+90) {myRobot.ArcadeDrive(0.0, 0.5);}
  //   else if (!stageDone) {prevTime = currentTime; resetEncoders(); stageDone = true;}
  //   else {
  //     myRobot.ArcadeDrive(0.0, 0.0);
  //     if (currentTime - prevTime >= 0.5) {stage++; stageDone = false;}
  //   }
  // }
  // if (stage == 4) {
  //   if (avgDist < 15) {myRobot.ArcadeDrive(0.5, 0.0);}
  //   else if (!stageDone) {prevTime = currentTime; resetEncoders(); stageDone = true;}
  //   else {
  //     myRobot.ArcadeDrive(0.0, 0.0);
  //     if (currentTime - prevTime >= 0.5) {stage++; stageDone = false;}
  //   }
  // }
  // if (stage == 05) {
  //   if (heading() < 40+180 || heading() > 50+180) {myRobot.ArcadeDrive(0.0, 0.5);}
  //   else if (!stageDone) {prevTime = currentTime; resetEncoders(); stageDone = true;}
  //   else {
  //     myRobot.ArcadeDrive(0.0, 0.0);
  //     if (currentTime - prevTime >= 0.5) {stage++; stageDone = false;}
  //   }
  // }
  // if (stage == 6) {
  //   if (avgDist < 15) {myRobot.ArcadeDrive(0.5, 0.0);}
  //   else if (!stageDone) {prevTime = currentTime; resetEncoders(); stageDone = true;}
  //   else {
  //     myRobot.ArcadeDrive(0.0, 0.0);
  //     if (currentTime - prevTime >= 0.5) {stage++; stageDone = false;}
  //   }
  // }
  // if (stage == 07) {
  //   if (heading() < 40+270 || heading() > 50+270) {myRobot.ArcadeDrive(0.0, 0.5);}
  //   else if (!stageDone) {prevTime = currentTime; resetEncoders(); stageDone = true;}
  //   else {
  //     myRobot.ArcadeDrive(0.0, 0.0);
  //     if (currentTime - prevTime >= 0.5) {stage++; stageDone = false;}
  //   }
  // }
  

  //The following is the auto for the first Obstacle Course.
  //Uncomment it only when you're about to use it, then comment it out again.
  //Start at (3.625, 8.875) with heading 0
  //Fastest time = 21 second (DO NOT DELETE)
  // if (stage == 0) {//arrives at position 2 (14.125, 8.875) with heading 0
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 8.16200375+1) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 1) {//face down (turn right)
  //   myRobot.ArcadeDrive(0.0, autoturn);
  //   if (heading() >= 85-45 && heading() <= 95-45) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 2) {//arrives at position 3 (14.125, 3.625) with heading 90
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 5.239698-2) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 3) {//face left (turn right)
  //   myRobot.ArcadeDrive(0.0, autoturn);
  //   if (heading() >= 175-45 && heading() <= 185-45) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 4) {//arrives at position 4 (11.125, 3.625) with heading 180
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 3.340756) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 5) {//face up (turn right)
  //   myRobot.ArcadeDrive(0.0, autoturn);
  //   if (heading() >= 265-44 && heading() <= 275-44) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 6) {//arrives at position 5 (11.125, 6.625) with heading 270
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 2.907335) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 7) {//face right (turn right)
  //   myRobot.ArcadeDrive(0.0, autoturn);
  //   if (heading() <= 365-43 && heading() >= 355-43) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 8) {//arrives at position 6 (21.375, 6.625) with heading 0
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 10.217183) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 9) {//face up (turn left)
  //   myRobot.ArcadeDrive(0.0, -autoturn);
  //   if (heading() <= 275+43 && heading() >= 265+43) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 10) {//arrives at position 7 (21.375, 11.375) with heading 270
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 4.842042-2) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 11) {//face up (turn left)
  //   myRobot.ArcadeDrive(0.0, -autoturn);
  //   if (heading() <= 185+46.5 && heading() >= 175+46.5) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 12) {//arrives at position 7 (21.375, 11.375) with heading 270
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 4) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 13) {//face up (turn left)
  //   myRobot.ArcadeDrive(0.0, -autoturn);
  //   if (heading() <= 95+45 && heading() >= 85+45) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 14) {//arrives at position 7 (21.375, 11.375) with heading 270
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 8) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 15) {//face up (turn left)
  //   myRobot.ArcadeDrive(0.0, -autoturn);
  //   if (heading() <= 49 && heading() >= 39) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 16) {
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 9) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 17) {
  //   myRobot.ArcadeDrive(0.0, -autoturn);
  //   if (heading() <= 280+43 && heading() >= 270+43) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 18) {
  //   myRobot.ArcadeDrive(autospeed, 0.0);
  //   if (avgDist >= 3.25) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 19) {
  //   myRobot.ArcadeDrive(0.0, -autoturn);
  //   if (heading() <= 185+45 && heading() >= 175+45) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 20) {
  //   myRobot.ArcadeDrive(autospeed, 0.0);
  //   if (avgDist >= 30) {stage++; prevTime = currentTime; resetEncoders();}
  // }

  // The following is the auto for the second Obstacle Course.
  // Uncomment it only when you're about to use it, then comment it out again.
  //starts at (3.875, 3.625)
  // if (stage == 0) { //heading = 30 degrees
  //   myRobot.ArcadeDrive(0.0, -autoturn); //turn left 30 degrees
  //   if (heading() >= 29 || heading() <= 31) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 1) { //move to position 2 (8.768, 6.5)
  //   myRobot.ArcadeDrive(-1.0, 0.0);
  //   if (avgDist >= 5.15) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 2) { //heading = 0
  //   myRobot.ArcadeDrive(0.0, -0.7);
  //   if (heading() >= 359 || heading() <= 1) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 3) { //move to position 3 (21.375, 6.5)
  //   myRobot.ArcadeDrive(-1.0, 0,0);
  //   if (avgDist >= 12.107) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 4) { //heading = 90
  //   myRobot.ArcadeDrive(0.0, -0.7); //turn right 90 degrees
  //   if (heading() >= 89 && heading() <= 91) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 5) { //move to position 4 (21.375, 3.675)
  //   myRobot.ArcadeDrive(-1.0, 0.0);
  //   if (avgDist >= 2.325) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 6) { //turn left 90 degrees (heading = 0)
  //   myRobot.ArcadeDrive(0.0, 0.7);
  //   if (heading() >= 359 || heading() <= 1) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 7) { //move to position 5 (26.375, 3.675)
  //   myRobot.ArcadeDrive(-1.0, 0.0);
  //   if (avgDist >= 4.5)  {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 8) {//turn left 90 degrees (heading = 270)
  //   myRobot.ArcadeDrive(0.0, 0.7);
  //   if (heading() > 269 && heading() < 271) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 9) { //move to position 6 (26.375, 6.275)
  //   myRobot.ArcadeDrive(-1.0, 0.0);
  //   if (avgDist >= 2.25)  {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 10) { //turn left 90 degrees (heading = 180)
  //   myRobot.ArcadeDrive(0.0, 0.7);
  //   if (heading() > 175 && heading() < 185) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 11) { //move to position 7 (23.625, 6.275)
  //   myRobot.ArcadeDrive(-1.0, 0.0);
  //   if (avgDist >= 2.25) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 12) {//turn left until heading = 240
  //   myRobot.ArcadeDrive(0.0, 0.7);
  //   if (heading() > 235 && heading() < 245) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 13) { //move to position 8 (22.025, 3.5)
  //   myRobot.ArcadeDrive(-1.0, 0.0);
  //   if (avgDist >= 2.82) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 14) { //turn right until heading = 180
  //   myRobot.ArcadeDrive(0.0, -0.7);
  //   if (heading() > 179 && heading() < 181) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 15) {  //move to position 9 (8.625, 3.5)
  //   myRobot.ArcadeDrive(-1.0, 0.0);
  //   if (avgDist >= 12.9) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 16) { //turn right to a heading of 202.479 degrees
  //   myRobot.ArcadeDrive(0.0, -0.7);
  //   if (heading() >= 201.479 && heading() <= 203.479) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 17) { //drive into the endzone victorious
  //   myRobot.ArcadeDrive(-1.0, 0.0);
  //   if (avgDist >= 9) {stage++; prevTime = currentTime; resetEncoders();}
  // }
}

void Robot::TeleopInit() {
  frontLeft->SetSelectedSensorPosition(0.0);
  backLeft->SetSelectedSensorPosition(0.0);
  frontRight->SetSelectedSensorPosition(0.0);
  backRight->SetSelectedSensorPosition(0.0);
  timer.Reset();
  timer.Start();
  shootTimer.Reset();
  turn = 0.0;
  speed = 0.0;
  //sensitivity = -two.GetRawAxis(1);
  ballIn.Set(frc::DoubleSolenoid::Value::kOff);//piston1 no go nyoo
  compressor->Start();
  ballStorage.Set(frc::DoubleSolenoid::Value::kReverse);
  calibratePigeon();
}

void Robot::TeleopPeriodic() {
  avgDist = -(-(double)frontLeft->GetSelectedSensorPosition()-(double)backLeft->GetSelectedSensorPosition()+(double)frontRight->GetSelectedSensorPosition()+(double)backRight->GetSelectedSensorPosition())/4.0;
  avgDist /= 6612.5;
  frc::SmartDashboard::PutNumber("Average Distance:", avgDist);
  frc::SmartDashboard::PutNumber("Heading x: ", xyz[0]);
  frc::SmartDashboard::PutNumber("Heading y: ", xyz[1]);
  compressor->SetClosedLoopControl(true);
  //increase sensitivity with the right bumper
 //In order to go backwards do piston.Set(DoubleSolenoid::Value::kReverse);
 if (one.GetRawButton(8)) {
   ballStorage.Set(frc::DoubleSolenoid::Value::kForward);
 }
 else {
   ballStorage.Set(frc::DoubleSolenoid::Value::kReverse);
 }
 if(one.GetRawAxis(3)>0.1){
   intake.Set(0.4);
 }else if(one.GetRawAxis(2)>0.1){
   intake.Set(-1.0);
 }else{
   intake.Set(0.0);
 }


//  if(one.GetRawButton(2)){
//    ballUnstuck.Set(true);
//  }else{
//    ballUnstuck.Set(false);
//  }

 double outtakeSpeed = -1.0;

  // if (one.GetRawButton(3)) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kForward);
  // }
  // else if (one.GetRawButton(4)) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kReverse);
  // }
  // else if (one.GetRawButton(9)) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kOff);
  // }

  if (one.GetRawButton(2)) {
    top.Set(-0.45);
    bottom.Set(0.3);
  }
  else if (!one.GetRawButton(2)&&one.GetRawButton(3)) {
    top.Set(-0.4);
    bottom.Set(0.78);
  }
  else if (!one.GetRawButton(2)&&!one.GetRawButton(3)&&one.GetRawButton(4)) {
    top.Set(-0.35);
    bottom.Set(0.3);
  }
  else if (!one.GetRawButton(2) && !one.GetRawButton(3)&&!one.GetRawButton(4)) {
    top.Set(0.0);
    bottom.Set(0.0);
  }
  
  if(one.GetRawButton(6)) {
    ballIn.Set(frc::DoubleSolenoid::Value::kForward);//piston1 go 
  }
  else if (!one.GetRawButton(6)&&one.GetRawButton(5)) {
    ballIn.Set(frc::DoubleSolenoid::Value::kReverse);//piston1 go shwoop
  }
  else if (!one.GetRawButton(6)&&!one.GetRawButton(5)){
    ballIn.Set(frc::DoubleSolenoid::Value::kOff);//piston1 stop
  }
  //sensitivity = -two.GetRawAxis(1);
  
  /*

 //sensitivity = one.GetRawAxis(2);
//  if (one.GetPOV(0) && sensitivity < 1.0) {
//    sensitivity += 0.01;
//  }
//  else if (one.GetPOV(0) && sensitivity >= 1.0) {
//    sensitivity = 1.0;
//  }
//  else if (one.GetPOV(180) && sensitivity > 0.0) {
//    sensitivity -= 0.01;
//  }
//  else if (one.GetPOV(180) && sensitivity <= 0.0) {
//    sensitivity = 0.0;
//  }*/
  speed = one.GetRawAxis(1);
  turn = one.GetRawAxis(4);

  myRobot.ArcadeDrive(speed*sensitivity, turn*sensitivity);

  pan.Set(trueMap(two.GetRawAxis(0),1,-1,1,0));
  tilt.Set(trueMap(two.GetRawAxis(1),-1,1,1,0));
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif