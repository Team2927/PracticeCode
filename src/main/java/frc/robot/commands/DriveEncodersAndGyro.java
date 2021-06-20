/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DriveEncodersAndGyro extends Command {

  double error = 0;
  double desiredLocation = 0; //degrees
  double currentLocation = 0; //degrees
  double pAdjustment = 0;
  double iAdjustment = 0;
  double dAdjustment = 0;
  double pidAdjustment = 0;
  double distanceTraveled = 0; //inches
  double lastError = 0;
  double motorSpeed = 0;
  double distance = 0;

  public DriveEncodersAndGyro(double motorSpeed, double distanceInInches, double headingInDegrees) {
    requires(Robot.m_drive);
    this.motorSpeed = motorSpeed;
    this.distance = distanceInInches;
    this.distance = this.distance * RobotMap.encoderTicksToInchesConversion;
    this.distance = (int) this.distance;
    this.desiredLocation = headingInDegrees;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_drive.stop();
    Robot.m_drive.resetLeft();
    Robot.m_drive.gyroReset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    error = desiredLocation - currentLocation;
    pAdjustment = error * RobotMap.pStraightConstant * RobotMap.gainStraightConstant;
    iAdjustment = iAdjustment + (error * RobotMap.iStraightConstant * RobotMap.gainStraightConstant);
    dAdjustment = (error - lastError) * RobotMap.dStraightConstant * RobotMap.gainStraightConstant;
    lastError = error;
    pidAdjustment = pAdjustment + iAdjustment + dAdjustment;
    distanceTraveled = Robot.m_drive.encoderGetLeft();
    Robot.m_drive.tankDrive((motorSpeed - pidAdjustment), (-motorSpeed - pidAdjustment));
    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(Robot.m_drive.encoderGetLeft()) >= (Math.abs(this.distance)));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.m_drive.stop();
  }
}
