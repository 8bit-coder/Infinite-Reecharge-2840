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
#include <frc/DriverStation.h>
#include <units/units.h>

#include "rev/SparkMax.h"
#include <frc/Compressor.h>
#include <frc/Talon.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/I2C.h>
// #include <frc/AddressableLED.h>
#include <math.h>

// frc::PowerDistributionPanel pdp = new frc::PowerDistributionPanel();
cs::UsbCamera camera0;
// cs::UsbCamera camera1;
cs::VideoSink server;
frc::Joystick one{0}, two{1};
//rev::SparkMax intake{4}, outtake{5};
rev::SparkMax intake{0};
frc::Servo pan{6},tilt{7};
int stage = 0;
double xyz[] = {0.0, 0.0, 0.0};
// frc::Talon frontLeft{2}, frontRight{0}, backRight{3}, backLeft{1}, out{8};

ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontLeft = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(2);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontRight = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(1);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *backLeft= new ctre::phoenix::motorcontrol::can::WPI_TalonFX(3);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *backRight = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(0);
ctre::phoenix::motorcontrol::can::TalonSRX *talon = new ctre::phoenix::motorcontrol::can::TalonSRX(10);

ctre::phoenix::motorcontrol::can::WPI_TalonFX *top = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(4);
ctre::phoenix::motorcontrol::can::WPI_TalonFX *bottom = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(5);
double outtakeSpeedHigh = 0, outtakeSpeedLow = 0, outtakeVoltHigh = 0, outtakeVoltLow = 0;
int smoothing = 6;


frc::RobotDrive myRobot{*frontLeft, *backLeft, *frontRight, *backRight};
frc::Timer timer, shootTimer;

frc::I2C *i2c = new frc::I2C(frc::I2C::I2C::Port::kOnboard, 0x54); //Declare arduino as a I2C device with address 0x54
//frc::SendableChooser autoChoice;
frc::Solenoid ballUnstuck{0};
frc::DoubleSolenoid ballIn{3, 7}, ballStorage{2, 1};
frc::Compressor *compressor = new frc::Compressor(0);

ctre::phoenix::sensors::PigeonIMU pigeon{talon};
//0.65 is the ideal sensitivity
double speed = 0.0, turn = 0.0, autoturn = 0.5, sensitivity = 1.0, turnKey, avgDist = 0.0, currentTime = 0.0, prevTime = 0.0, maxTime = 0, maxSpeed = 0;
bool isUpPressed, isDownPressed, redAuto;
double sP,tN;
int16_t accel[3];

static constexpr int kLength = 278;


double voltage, oldVoltage, olderVoltage, oldestVoltage, averageVoltage, previousAverageVoltage;
uint8_t readData[10];
double outtakeSpeed1A = -0.45;
double outtakeSpeed1B = 0.4;
double outtakeSpeed2A = -0.4;
double outtakeSpeed2B = 0.78;
double outtakeSpeed3A = -0.5;
double outtakeSpeed3B = 0.5;
double autospeed = 0.65;
bool aborted = false;

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
  // pigeon.SetCompassAngle(0.0);
  // pigeon.SetCompassDeclination(0);
  pigeon.SetFusedHeading(0);
  // pigeon.SetFusedHeadingToCompass(0);
  pigeon.SetYaw(0);
  // pigeon.SetYawToCompass(0);
}

void Robot::RobotInit() {
  camera0 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
  // camera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
  server = frc::CameraServer::GetInstance()->GetServer();
  camera0.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
  // camera1.SetConnectionStrategy(cs::VideoSource::ConnectionStrategy::kConnectionKeepOpen);
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
  top->ConfigFactoryDefault();
  bottom->ConfigFactoryDefault();
}

void Robot::RobotPeriodic() {
  pigeon.GetAccumGyro(xyz);
  aborted = i2c->ReadOnly(10, readData); //store 10 bytes to the readData array
  // frc::SmartDashboard::PutNumber("Heading z: ", heading());
  // frc::SmartDashboard::PutNumber("Timer", timer.Get());
  // frc::SmartDashboard::PutNumber("FrontLeft Distance: ", (double)frontLeft->GetSelectedSensorPosition()/6612.5);
  // frc::SmartDashboard::PutNumber("FrontRight Distance: ", (double)frontRight->GetSelectedSensorPosition()/6612.5);
  // frc::SmartDashboard::PutNumber("Backleft Distance: ", (double)backLeft->GetSelectedSensorPosition()/6612.5);
  // frc::SmartDashboard::PutNumber("Backright Distance: ", (double)backRight->GetSelectedSensorPosition()/6612.5);
  // frc::SmartDashboard::PutNumber("FrontLeft Velocity: ", (double)frontLeft->GetSelectedSensorVelocity()/6612.5);
  // frc::SmartDashboard::PutNumber("FrontRight Velocity: ", (double)frontRight->GetSelectedSensorVelocity()/6612.5);
  // frc::SmartDashboard::PutNumber("Backleft Velocity: ", (double)backLeft->GetSelectedSensorVelocity()/6612.5);
  // frc::SmartDashboard::PutNumber("Backright Velocity: ", (double)backRight->GetSelectedSensorVelocity()/6612.5);
  // frc::SmartDashboard::PutNumber("Top Velocity: ", (double)top->GetSelectedSensorVelocity());
  // frc::SmartDashboard::PutNumber("Bottom Velocity: ", (double)bottom->GetSelectedSensorVelocity());
  // frc::SmartDashboard::PutNumber("Heading: ",pigeon.GetAbsoluteCompassHeading());
  // frc::SmartDashboard::PutNumber("outtake speed 3A ", outtakeSpeed3A);
  // frc::SmartDashboard::PutNumber("outtake speed 3B ", outtakeSpeed3B);
  frc::SmartDashboard::PutBoolean("i2c to arduino aborted: ", aborted);
  // frc::SmartDashboard::PutBoolean("aborted: ", aborted); //determine whether or not it's aborted and display on dashboard
  // frc::SmartDashboard::PutNumber("unmapped x of ball: ", readData[0]); //display each value given by arduino
  // frc::SmartDashboard::PutNumber("mapped x of ball: ", readData[1]);
  // frc::SmartDashboard::PutNumber("readData 2: ", readData[2]);
  // frc::SmartDashboard::PutNumber("mapped y of ball: ", readData[3]);
  // frc::SmartDashboard::PutNumber("readData 4: ", readData[4]);
  // frc::SmartDashboard::PutNumber("readData 5: ", readData[5]);
  // frc::SmartDashboard::PutNumber("readData 6: ", readData[6]);
  // frc::SmartDashboard::PutBoolean("Ball on left: ", readData[7]);
  // frc::SmartDashboard::PutBoolean("Ball on right: ", readData[8]);
  frc::SmartDashboard::PutNumber("items seen ", readData[9]);

  
  // Rainbow();
  // m_led.SetData(m_ledBuffer);
}

void Robot::AutonomousInit() {
  myRobot.SetSafetyEnabled(false);
  if (readData[9] > 0) {redAuto = true;}
  else {redAuto = false;}
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
}



void Robot::AutonomousPeriodic() {
  avgDist = (-(double)frontLeft->GetSelectedSensorPosition()-(double)backLeft->GetSelectedSensorPosition()+(double)frontRight->GetSelectedSensorPosition()+(double)backRight->GetSelectedSensorPosition())/4.0;
  avgDist /= 6612.5;
  // frc::SmartDashboard::PutNumber("Timer", timer.Get());
  frc::SmartDashboard::PutNumber("Average Distance", avgDist);
  // frc::SmartDashboard::PutNumber("Stage Time: ", currentTime-prevTime);
  frc::SmartDashboard::PutNumber("Stage: ", stage+1);
  // currentTime = timer.Get();
  // frc::SmartDashboard::PutNumber("Heading: ", heading());
  //best time is 16.57 not including penalties, 26.57 including penalties
  double currentHeading = heading();
  switch(stage) {
  case 0:
    autospeed = -0.5; //set autospeed
    autoturn = 0.4;
    compressor->Stop();
    ballIn.Set(frc::DoubleSolenoid::Value::kReverse); //extend intake
    stage++; prevTime = currentTime; resetEncoders();
    break;
  case 1:
    myRobot.ArcadeDrive(autospeed, 0.08); // drive for 5 feet
    if (avgDist >= 4.5) {
      stage++; prevTime = currentTime; resetEncoders();
  }
    break;
  case 2:
    myRobot.ArcadeDrive(0.0, -autoturn); // turn 45 degrees right (heading = 45)
    if (currentHeading >= 275 + 10 && currentHeading <= 275 + 20) {
      stage++; prevTime = currentTime; resetEncoders();
  }
    break;
  case 3:
    myRobot.ArcadeDrive(autospeed, 0.0); //drive for a lil over 10.5 ft
    if (avgDist >= 5) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 4:
    myRobot.ArcadeDrive (0.0, autoturn); // turn 90 degrees left (heading = 315)
    if (currentHeading >= 345 && currentHeading <= 355) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 5: 
    myRobot.ArcadeDrive(autospeed, 0.0); // drive for 7.07107 ft
    if (avgDist >= 11.6) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 6:
    myRobot.ArcadeDrive(0.0, autoturn);
    if (currentHeading <= 47 && currentHeading >= 37) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 7: 
    myRobot.ArcadeDrive(autospeed, 0.0); // drive for 7.07107 ft
    if (avgDist >= 5.83-.4) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 8:
    myRobot.ArcadeDrive(0.0, -autoturn);
    if (currentHeading >= 15 && currentHeading <= 25) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 9:
    myRobot.ArcadeDrive(autospeed, 0.0);
    if (avgDist >= 2.3) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 10:
    myRobot.ArcadeDrive(0.0, -autoturn);
    if (currentHeading <= 275 + 35 && currentHeading >= 275 + 25) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 11:
    myRobot.ArcadeDrive(autospeed, 0.0);
    if (avgDist >= 3.5) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 12:
    myRobot.ArcadeDrive(0.0, -autoturn);
    if (currentHeading >= 180 + 20 && currentHeading <= 180 + 30) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 13:
    myRobot.ArcadeDrive(autospeed, 0.0);
    if (avgDist >= 5) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 14:
    myRobot.ArcadeDrive(0.0, -autoturn);
    if (currentHeading <= 90 + 32 && currentHeading >= 90 + 22) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 15:
    myRobot.ArcadeDrive(autospeed, 0.0);
    if (avgDist >= 2.68+1.75) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 16:
    myRobot.ArcadeDrive(0.0, autoturn);
    if (currentHeading <= 180 - 9 && currentHeading >= 180 - 19) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 17:
    myRobot.ArcadeDrive(autospeed, 0.0);
    if (avgDist >= 13) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
  case 18:
    myRobot.ArcadeDrive(0.0, autoturn);
    if (currentHeading >= 187+22.5 && currentHeading <= 197+22.5) {
      stage++; prevTime = currentTime; resetEncoders();
    }
    break;
    case 19:
    myRobot.ArcadeDrive(autospeed, 0.0);
    if (avgDist >= 10) {
      stage++; prevTime = currentTime; resetEncoders();
    }
}
  //blue auto for map A:
//red auto for map A: (best = 7.5 seconds)
// if (redAuto) {
//   switch(stage) {
//   case 0:
//     autospeed = -0.5; //set autospeed
//     autoturn = 0.45;
//     compressor->Start(); //start compressor
//     ballIn.Set(frc::DoubleSolenoid::Value::kForward); //extend intake
//     stage++; prevTime = currentTime; resetEncoders();
//     break;
//   case 1:
//     myRobot.ArcadeDrive(autospeed, 0.08); // drive for 5 feet
//     intake.Set(0.4); //intake one ball on the way
//     if (avgDist >= 5) {
//       stage++; prevTime = currentTime; resetEncoders();
//   }
//     break;
//   case 2:
//     myRobot.ArcadeDrive(0.0, autoturn); // turn 45 degrees right (heading = 45)
//     if (heading() >= 26.565 - 19 && heading() <= 26.565 - 11.5) {
//       stage++; prevTime = currentTime; resetEncoders();
//   }
//     break;
//   case 3:
//     myRobot.ArcadeDrive(autospeed, 0.0); //drive for a lil over 10.5 ft
//     intake.Set(0.4); //intake one ball on the way
//     if (avgDist >= 5.590) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 4:
//     myRobot.ArcadeDrive (0.0, -autoturn); // turn 90 degrees left (heading = 315)
//     intake.Set(0.4);
//     if (heading() >= 288.435+18 && heading() <= 288.435+28) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 5: 
//     myRobot.ArcadeDrive(autospeed, 0.0); // drive for 7.07107 ft
//     intake.Set(0.4); //pick up a ball on the way
//     if (avgDist >= 7.906-1) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 6:
//     myRobot.ArcadeDrive(0.0, autoturn);
//     if (heading() <= 360 - 10 && heading() >= 360 - 20) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 7: 
//     myRobot.ArcadeDrive(-1.0, 0.0); // drive for 7.07107 ft
//     if (avgDist >= 15) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   }
// }
// else {
//   switch(stage) {
//   case 0:
//     autospeed = -0.5; //set autospeed
//     autoturn = 0.45;
//     compressor->Start(); //start compressor
//     ballIn.Set(frc::DoubleSolenoid::Value::kForward); //extend intake
//     stage++; prevTime = currentTime; resetEncoders();
//     break;
//   case 1:
//     myRobot.ArcadeDrive(autospeed, 0.08); // drive for 5 feet
//     intake.Set(0.4); //intake one ball on the way
//     if (avgDist >= 12.5) {
//       stage++; prevTime = currentTime; resetEncoders();
//   }
//     break;
//   case 2:
//     myRobot.ArcadeDrive(0.0, -autoturn); // turn 45 degrees right (heading = 45)
//     if (heading() >= 296.565+8 && heading() <= 296.565+18) {
//       stage++; prevTime = currentTime; resetEncoders();
//   }
//     break;
//   case 3:
//     myRobot.ArcadeDrive(autospeed, 0.0); //drive for a lil over 10.5 ft
//     intake.Set(0.4); //intake one ball on the way
//     if (avgDist >= 2.236+2.5) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 4:
//     myRobot.ArcadeDrive (0.0, autoturn); // turn 90 degrees left (heading = 315)
//     if (heading() <= 350-8 && heading() >= 340-8) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 5: 
//     myRobot.ArcadeDrive(-1.0, 0.0); // drive for 7.07107 ft
//     intake.Set(0.4); //pick up a ball on the way
//     if (avgDist >= 6) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
// }
// }
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
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 3.25) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 19) {
  //   myRobot.ArcadeDrive(0.0, -autoturn);
  //   if (heading() <= 185+45 && heading() >= 175+45) {stage++; prevTime = currentTime; resetEncoders();}
  // }
  // else if (stage == 20) {
  //   myRobot.ArcadeDrive(0.5, 0.0);
  //   if (avgDist >= 30) {stage++; prevTime = currentTime; resetEncoders();}
  // }
//second auto stuff (tom version)
/*if (stage == 0) { //heading = 30 degrees
  myRobot.ArcadeDrive(0.0, -autoturn); //turn left 30 degrees
  if (heading() >= 342.5 && heading() <= 347.5) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 1) { //move to position 2 (8.768, 6.5)
  myRobot.ArcadeDrive(autospeed, 0.0);
  if (avgDist >= 5.15) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 2) { //heading = 0
  myRobot.ArcadeDrive(0.0, autoturn);
  if (heading() >= 342.5 && heading() <= 347.5) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 3) { //move to position 3 (21.375, 6.5)
  myRobot.ArcadeDrive(autospeed, 0,0);
  if (avgDist >= 12.107) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 4) { //heading = 90
  myRobot.ArcadeDrive(0.0, autoturn); //turn right 90 degrees
  if (heading() >= 90-50 && heading() <= 90-40) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 5) { //move to position 4 (21.375, 3.675)
  myRobot.ArcadeDrive(autospeed, 0.0);
  if (avgDist >= 2.325) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 6) { //turn left 90 degrees (heading = 0)
  myRobot.ArcadeDrive(0.0, -autoturn);
  if (heading() >= 360-50 && heading() <= 360-40) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 7) { //move to position 5 (26.375, 3.675)
  myRobot.ArcadeDrive(autospeed, 0.0);
  if (avgDist >= 4.5)  {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 8) {//turn left 90 degrees (heading = 270)
  myRobot.ArcadeDrive(0.0, -autoturn);
  if (heading() >= 270-50 && heading() <= 270-40) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 9) { //move to position 6 (26.375, 6.275)
  myRobot.ArcadeDrive(autospeed, 0.0);
  if (avgDist >= 2.25)  {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 10) { //turn left 90 degrees (heading = 180)
  myRobot.ArcadeDrive(0.0, -autoturn);
  if (heading() >= 180-50 && heading() <= 180-40) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 11) { //move to position 7 (23.625, 6.275)
  myRobot.ArcadeDrive(autospeed, 0.0);
  if (avgDist >= 2.25) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 12) {//turn left until heading = 240
  myRobot.ArcadeDrive(0.0, -autoturn);
  if (heading() >= 205 && heading() <= 215) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 13) { //move to position 8 (22.025, 3.5)
  myRobot.ArcadeDrive(autospeed, 0.0);
  if (avgDist >= 2.82) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 14) { //turn right until heading = 180
  myRobot.ArcadeDrive(0.0, autoturn);
  if (heading() >= 180-50 && heading() <= 180-40) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 15) {//move to position 9 (8.625, 3.5)
  myRobot.ArcadeDrive(autospeed, 0.0);
  if (avgDist >= 12.9) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 16) { //turn right to a heading of 202.479 degrees
  myRobot.ArcadeDrive(0.0, autoturn);
  if (heading() >= 202.479-50 && heading() <= 202.479-40) {stage++; prevTime = currentTime; resetEncoders();}
}
else if (stage == 17) { //drive into the endzone victorious
  myRobot.ArcadeDrive(autospeed, 0.0);
  if (avgDist >= 9) {stage++; prevTime = currentTime; resetEncoders();}
}
*/

// -----------------------------------------------------------
// second auto stuff nort boi redux (Aaron and Lucas version) 
// ------------------------------------------------------------
// powerfuck search path b reds 
// if (pixy sees ball) 
// The recorded time is 5.03 seconds for this autonomous.
// if (redAuto) {
// switch(stage) {
//   case 0:
//     autospeed = -0.60; //set autospeed
//     autoturn = 0.45;
//     compressor->Start(); //start compressor
//     ballIn.Set(frc::DoubleSolenoid::Value::kForward); //extend intake
//     stage++; prevTime = currentTime; resetEncoders();
//     break;
//   case 1:
//     myRobot.ArcadeDrive(autospeed, 0.02); // drive for 5 feet
//     intake.Set(0.4); //intake one ball on the way
//     if (avgDist >= 2.5) {
//       stage++; prevTime = currentTime; resetEncoders();
//   }
//     break;
//   case 2:
//     myRobot.ArcadeDrive(0.0, autoturn); // turn 45 degrees right (heading = 45)
//     if (heading() >= 45 - 36 && heading() <= 45-27) {
//       stage++; prevTime = currentTime; resetEncoders();
//   }
//     break;
//   case 3:
//     myRobot.ArcadeDrive(autospeed, 0.0); //drive for a lil over 10.5 ft
//     intake.Set(0.4); //intake one ball on the way
//     if (avgDist >= 7.07107) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 4:
//     myRobot.ArcadeDrive (0.0, -autoturn); // turn 90 degrees left (heading = 315)
//     if (heading() >= 344 && heading() <= 354) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 5: 
//     myRobot.ArcadeDrive(autospeed, 0.0); // drive for 7.07107 ft
//     intake.Set(0.4); //pick up a ball on the way
//     if (avgDist >= 6) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 6:
//     myRobot.ArcadeDrive(0.0, autoturn); //turn 45 degrees right (heading = 0)
//     if (heading() >= 360-40 && heading() <= 360-30) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//   break;
//   case 7:
//     myRobot.ArcadeDrive(-1.0, 0.0); //zoom forward 13 feet
//     if (avgDist >= 13) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
// }
// }
//powerfuck search path b bluess I'm probably doing this wrong btw but whatevs
// else if (pixy does not see ball)
// else {
// switch(stage) {
//   case 0:
//     autospeed = -0.525; //set autospeed
//     autoturn = 0.4;
//     compressor->Start(); //start compressor
//     ballIn.Set(frc::DoubleSolenoid::Value::kForward); //extend intake
//     stage++; prevTime = currentTime; resetEncoders();
//     break;
//   case 1:
//     myRobot.ArcadeDrive(autospeed, 0.09); // drive for 10 feet
//     intake.Set(0.4);
//     if (avgDist >= 11) {
//       stage++; prevTime = currentTime; resetEncoders();
//   }
//     break;
//   case 2:
//     myRobot.ArcadeDrive(0.0, -autoturn); // turn 45 degrees left (heading = 315)
//     if (heading() >= 315+20 && heading() <= 315+30) {
//       stage++; prevTime = currentTime; resetEncoders();
//   }
//     break;
//   case 3:
//     myRobot.ArcadeDrive(autospeed, 0.0); //drive for 12.5 feet
//     intake.Set(0.4); //intake two balls on the way
//     if (avgDist >= 7.075) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 4:
//     myRobot.ArcadeDrive (0.0, autoturn); // turn 90 degrees right (heading = 45)
//     if (heading() >= 40-7 && heading() <= 50-7) {
//       stage++; prevTime = currentTime; resetEncoders();
//     }
//     break;
//   case 5: 
//     myRobot.ArcadeDrive(-1.0, 0.0); // drive for a little over 10 feet and hit endzone yay!
//     intake.Set(0.4); //pick up a ball on the way
//     if (avgDist >= 11.5) {
//       stage = 0; prevTime = currentTime; resetEncoders();
//     }
//     break;
// }
// }
// if (stage == 0) {
// myRobot.ArcadeDrive(autospeed, 0.0);
// if (avgDist >= 3.625) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 1) { //heading = 270 degrees
//   myRobot.ArcadeDrive(0.0, -autoturn); //turn left 90 degrees
//   if (heading() >= 310 && heading() <= 320) {stage++; prevTime = currentTime; resetEncoders();}
// }
// if (stage == 2) {
//   myRobot.ArcadeDrive(autospeed, 0.0);
//   if (avgDist >= 3.625) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 3) { //heading = 270 degrees
//   myRobot.ArcadeDrive(0.0, -autoturn); 
//   if (heading() >= 255 && heading() <= 260) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 4) { 
//   myRobot.ArcadeDrive(-autospeed, 0.0); 
//   if (avgDist <= -10.54092553) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 5) {
//   myRobot.ArcadeDrive(0.0, -autoturn);
//   if (heading() >= 210 && heading() <= 220) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 6) {
//     myRobot.ArcadeDrive(-autospeed, 0.0);
//     if (avgDist <= -4.33772234) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 7) {
//     myRobot.ArcadeDrive(0.0, -autoturn);
//     if (heading() >= 130 && heading() <= 140) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 8) {
//   myRobot.ArcadeDrive(-autospeed, 0.0); 
//   if (avgDist <= -3.33772234) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 9) {//////////////////////////////////////////
//   myRobot.ArcadeDrive(autospeed, 0.0);
//   if (avgDist >= 10) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 10) {
//     myRobot.ArcadeDrive(0.0, -autoturn);
//     if (heading() >= 40 && heading() <= 50) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 9) {
//   myRobot.ArcadeDrive(autospeed, 0.0);
//   if (avgDist >= 7.5) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 10) {
//     myRobot.ArcadeDrive(0.0, -autoturn);
//     if (heading() >= 310 && heading() <= 320) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 11) {
//     myRobot.ArcadeDrive(autospeed, 0.0);
//     if (avgDist >= 10) {stage++; prevTime = currentTime; resetEncoders();}
// }
// else if (stage == 12) {
//     myRobot.ArcadeDrive(-autospeed, 0.0);
//     if (avgDist <= 5) {stage++; prevTime = currentTime; resetEncoders();}
// }
}


void Robot::TeleopInit() {
  top->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 10);
  bottom->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, 0, 10);
  bottom->SetInverted(ctre::phoenix::motorcontrol::TalonFXInvertType::CounterClockwise);
  top->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  top->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  bottom->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  bottom->SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  top->ConfigNominalOutputForward(0, 10);
  top->ConfigNominalOutputReverse(0, 10);
  top->ConfigPeakOutputForward(1, 10);
  top->ConfigPeakOutputReverse(-1, 10);

  top->SelectProfileSlot(0, 0);
  top->Config_kF(0, 0.3, 10);
  top->Config_kP(0, 0.1, 10);
  top->Config_kI(0, 0.0, 10);
  top->Config_kD(0, 0.0, 10);

  /* Set acceleration and vcruise velocity - see documentation */
  top->ConfigMotionCruiseVelocity(1500, 10);
  top->ConfigMotionAcceleration(1500, 10);
  /* Zero the sensor */
  top->SetSelectedSensorPosition(0, 0, 10);

  bottom->ConfigNominalOutputForward(0, 10);
  bottom->ConfigNominalOutputReverse(0, 10);
  bottom->ConfigPeakOutputForward(1, 10);
  bottom->ConfigPeakOutputReverse(-1, 10);

  bottom->SelectProfileSlot(0, 0);
  bottom->Config_kF(0, 0.3, 10);
  bottom->Config_kP(0, 0.1, 10);
  bottom->Config_kI(0, 0.0, 10);
  bottom->Config_kD(0, 0.0, 10);

  /* Set acceleration and vcruise velocity - see documentation */
  bottom->ConfigMotionCruiseVelocity(1500, 10);
  bottom->ConfigMotionAcceleration(1500, 10);
  /* Zero the sensor */
  bottom->SetSelectedSensorPosition(0, 0, 10);  

  //smoothing things:
  bottom->ConfigMotionSCurveStrength(smoothing, 0);
  top->ConfigMotionSCurveStrength(smoothing, 0);
  
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
  voltage = frc::DriverStation::GetInstance().GetBatteryVoltage();
  averageVoltage = (voltage + oldVoltage + olderVoltage + oldestVoltage)/4;
  avgDist = -(-(double)frontLeft->GetSelectedSensorPosition()-(double)backLeft->GetSelectedSensorPosition()+(double)frontRight->GetSelectedSensorPosition()+(double)backRight->GetSelectedSensorPosition())/4.0;
  avgDist /= 6612.5;
  frc::SmartDashboard::PutNumber("Average Distance:", avgDist);
  // frc::SmartDashboard::PutNumber("Heading x: ", xyz[0]);
  // frc::SmartDashboard::PutNumber("Heading y: ", xyz[1]);
  // compressor->SetClosedLoopControl(true);
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

//frc::DriverStation::GetBatteryVoltage(); 


//  if(one.GetRawButton(2)){
//    ballUnstuck.Set(true);
//  }else{
//    ballUnstuck.Set(false);
//  }
if (averageVoltage < 11.5) {
  if (one.GetRawButton(4)) {
    myRobot.ArcadeDrive (0.0, 0.0);
  }
}
else {
  if (one.GetRawButton(4)) {
    myRobot.ArcadeDrive (0.0, 0.0); 
  }
}

 if (averageVoltage - previousAverageVoltage >= 0.5) {
   outtakeSpeed1A -= 0.02;
   outtakeSpeed1B += 0.02;
   outtakeSpeed2A -= 0.02;
   outtakeSpeed2B += 0.02;
   outtakeSpeed3A -= 0.02;
   outtakeSpeed3B += 0.02;
 }
 if (one.GetPOV() == 0) {
   outtakeSpeed3A -= 0.02;
   outtakeSpeed3B += 0.02;
 }

 else if (one.GetPOV() == 180) {
   outtakeSpeed3A += 0.02;
   outtakeSpeed3B -= 0.02;
 }

  // if (one.GetRawButton(3)) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kForward);
  // }
  // else if (one.GetRawButton(4)) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kReverse);
  // }
  // else if (one.GetRawButton(9)) {
  //   ballStorage.Set(frc::DoubleSolenoid::Value::kOff);
  // }

  // if (one.GetRawButton(2)) {
  //   top.Set(outtakeSpeed1A);
  //   bottom.Set(outtakeSpeed1B);
  // }
  // else if (!one.GetRawButton(2)&&one.GetRawButton(3)) {
  //   top.Set(outtakeSpeed2A);
  //   bottom.Set(outtakeSpeed2B);
  // }
  // else if (!one.GetRawButton(2)&&!one.GetRawButton(3)&&one.GetRawButton(4)) {
  //   top.Set(outtakeSpeed3A);
  //   bottom.Set(outtakeSpeed3B);
  // }
  // else if (!one.GetRawButton(2) && !one.GetRawButton(3)&&!one.GetRawButton(4)) {
  //   top.Set(0.0);
  //   bottom.Set(0.0);
  // }
//   if (one.GetRawButton(2)) {
//     outtakeSpeedHigh = 5;
//     outtakeSpeedLow = 5;
//   }
//   else if (one.GetRawButton(3)) {
//     outtakeSpeedHigh = 10;
//     outtakeSpeedLow = 10;
//   }
//   else if (one.GetRawButton(4)) {
//     outtakeSpeedHigh = 15;
//     outtakeSpeedLow = 15;
// }
//   else {
//     outtakeSpeedHigh = 0;
//     outtakeSpeedLow = 0;
// }

//   if (one.GetRawButton(2) || one.GetRawButton(3) || one.GetRawButton(4)) {
//     if (top->GetSelectedSensorVelocity()/2204.16666 < outtakeSpeedHigh) {
//       outtakeVoltHigh += 0.01;
//     }
//     else if (top->GetSelectedSensorVelocity()/2204.16666 > outtakeSpeedHigh) {
//       outtakeVoltHigh -= 0.01;
//     }
//     if (bottom->GetSelectedSensorVelocity()/2204.16666 < outtakeSpeedLow) {
//       outtakeVoltLow += 0.01;
//     }
//     else if (bottom->GetSelectedSensorVelocity()/2204.16666 > outtakeSpeedLow) {
//       outtakeVoltLow -= 0.01;
//     }
//     top->Set(outtakeVoltHigh);
//     bottom->Set(outtakeVoltLow);
//   }
//   else {
//     top->Set(0.0);
//     bottom->Set(0.0);
//   }
  if (one.GetRawButton(1)){
    top->Set(1.0);
    bottom->Set(-1.0);
    compressor->Stop();
  }
  else if (one.GetRawButton(2)) {
    top->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 5000);
    bottom->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, -12000);
    // bottom->Set(-1.0);
    // top->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 1000);
    compressor->Stop();
  }
  else if (one.GetRawButton(3)) {
    top->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 3000);
    bottom->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, -3000);
    compressor->Stop();
  }
  else if (one.GetRawButton(4)) {
    top->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 1225);
    bottom->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, -1225);
    compressor->Stop();
  }
  else if (one.GetRawButton(10)){
    top->Set(0);
    bottom->Set(0);
    compressor->Start();
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
oldVoltage = voltage;
olderVoltage = oldVoltage;
oldestVoltage = olderVoltage;
previousAverageVoltage = averageVoltage; 

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif