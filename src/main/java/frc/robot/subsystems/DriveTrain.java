/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveArcade;

public class DriveTrain extends Subsystem {

  private final DifferentialDrive driveWC = RobotMap.drive;

  Talon l = RobotMap.left1;
  Talon r = RobotMap.right1;
  Encoder lE = RobotMap.leftE;
  Encoder rE = RobotMap.rightE;
  ADXRS450_Gyro g = RobotMap.gyro;

  public void drive(double leftSpeed, double rightSpeed){
    l.set(leftSpeed);
    r.set(rightSpeed);
  }

  public void stop(){
    l.set(0.0);
    r.set(0.0);
  }

  public void arcadeDrive(double xSpeed, double yRotate){
    driveWC.arcadeDrive(xSpeed, yRotate);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    driveWC.tankDrive(leftSpeed, rightSpeed);
  }

  public void encodersReset(){
    resetLeft();
    resetRight();
  }

  public int encoderGetLeft(){
    return lE.get(); 
  }

  public int encoderGetRight(){
    return rE.get();
  }

  public void resetLeft(){
    lE.reset();
  }

  public void resetRight(){
    rE.reset();
  }

  public void gyroReset(){
    g.reset();
  }

  public void gyroCalibrate(){
    g.calibrate();
  }

  public void gyroRNC(){
    stop();
    gyroReset();
    gyroCalibrate();
  }

  public double gyroGetAngle(){
    return g.getAngle();
  }

  public double gyroGetRate(){
    return g.getRate();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveArcade());
  }
}
