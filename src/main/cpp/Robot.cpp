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

#include <frc/IterativeRobot.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/Spark.h>
#include <frc/Encoder.h>
#include <frc/WPILib.h>
#include <frc/PowerDistributionPanel.h>
#include <cameraServer/CameraServer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/SmartDashboard/SendableChooser.h>

#include "rev/SparkMax.h"
#include <frc/Compressor.h>
#include <frc/Talon.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <math.h>
using namespace frc;

frc::Joystick one{0},two{1};
frc::Talon Left{0},Right{1};
rev::SparkMax wheel{2},intake{3},outtake{4};
frc::DifferentialDrive myRobot{Left, Right};
frc::Timer timer;

//frc::SendableChooser autoChoice;
//Solenoid piston1{0};
DoubleSolenoid piston1{0,1},piston2{2,3};
Compressor compressor{0};
double speed, turn, sensitivity, turnKey;
bool isUpPressed, isDownPressed;
double sP,tN;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Timer", timer.Get());
  compressor.SetClosedLoopControl(false);
  compressor.Start();
  timer.Reset();
  timer.Start();
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  timer.Reset();
  timer.Start();
  turn = 0;
  speed = 0;
  sensitivity = 0.5;
}

void Robot::TeleopPeriodic() {
  //increase sensitivity with the right bumper
  /*
  piston.Set(true); makes the piston go
  piston.Set(false); makes the piston not go
  */
 //In order to go backwards do piston.Set(DoubleSolenoid::Value::kReverse);
 /*if(stick.GetRawButton(3)) {
    piston1.Set(true);
  }
  else{
    piston1.Set(false);
  }
  */

 if(one.GetRawButton(3)){
   intake.Set(-0.4);
 }else{
   intake.Set(0);
 }

 if(one.GetRawButton(1)){
   outtake.Set(1);
 }else{
   intake.Set(0);
 }

 if(two.GetRawButton(3)){
   wheel.Set(0.3);
 }else{
   wheel.Set(0);
 }

  if(two.GetRawButton(4)) {
    piston1.Set(DoubleSolenoid::Value::kForward);//piston1 go nyoom
    piston2.Set(DoubleSolenoid::Value::kForward);//piston2 go nyoom
  }
  else if (two.GetRawButton(5)) {
    piston1.Set(DoubleSolenoid::Value::kReverse);//piston1 go shwoop
    piston2.Set(DoubleSolenoid::Value::kReverse);//piston2 go shwoop
  }
  else{
    piston1.Set(DoubleSolenoid::Value::kOff);//piston1 stop
    piston2.Set(DoubleSolenoid::Value::kOff);//piston2 stop
  }
  sensitivity = -two.GetRawAxis(2);
  /*
  if (stick.GetRawButton(9) && sensitivity < 1.0) {
    sensitivity += 0.01;
  }
  else if (stick.GetRawButton(9)) {
    sensitivity += 0;
  } 
  else if (stick.GetRawButton(8) && sensitivity > 0.0) {
    sensitivity -= 0.01;
  }
  else if (stick.GetRawButton(8)) {
    sensitivity -= 0;
  }
  else {
    sensitivity = sensitivity;
  }
  if (sensitivity >= 1.0) {
    sensitivity = 1.0;
  }
  else if (sensitivity <= 0) {
    sensitivity = 0.0;
  }
  else {}
  
  
    //wheel.Set(0.3);
    //Solenoid.Set

  

  //turn with bumpers, too jittery
  /*if(stick.GetRawButton(7)){
     turn = (-1 * sensitivity);
  } else if(stick.GetRawButton(8)){
     turn = (1 * sensitivity);
  } else{
    turn = 0;
  }
  */
  if(one.GetRawAxis(0)>0.2||one.GetRawAxis(0)<-0.2){
			tN=one.GetRawAxis(0);
		}else{
      tN=0;
    }
  if(one.GetRawAxis(1)>0.2||one.GetRawAxis(1)<-0.2){
			sP=one.GetRawAxis(1);
  }else{
    sP=0;
  }
  speed = -sP * sensitivity;
  if (speed >= 0) {
    turn = ((tN * sensitivity)+(speed/4))+0.1;
  }
  else {
    turn = ((tN*sensitivity)-(speed/4))+0.15;
  }
  myRobot.ArcadeDrive(-speed, turn);
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
