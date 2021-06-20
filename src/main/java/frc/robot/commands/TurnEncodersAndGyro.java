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

public class TurnEncodersAndGyro extends Command {

  double error = 0;
  double desiredDegree = 0;
  double currentDegree = 0;
  double pAdjustment = 0;
  double iAdjustment = 0;
  double dAdjustment = 0;
  double pidAdjustment = 0;
  double motorSpeed = .55;
  double lastError = 0;
  double speed = 0;
  double way = 1;
  int n = 0;
  int i = 0;
  boolean pid = false;

  
  public TurnEncodersAndGyro(double changeInDegree){
    requires(Robot.m_drive);
    this.desiredDegree = changeInDegree;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    pid = false;
    way = 1;
    n = 0;
    i = 0;
    Robot.m_drive.stop();
    Robot.m_drive.gyroReset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentDegree = Robot.m_drive.gyroGetAngle();
    error = desiredDegree - currentDegree;
    pAdjustment = error * RobotMap.pTurnConstant * RobotMap.gainTurnConstant;
    iAdjustment = iAdjustment + (error * RobotMap.iTurnConstant * RobotMap.gainTurnConstant);
    dAdjustment = (error - lastError) * RobotMap.dTurnConstant * RobotMap.gainTurnConstant;
    lastError = error;
    pidAdjustment = pAdjustment + iAdjustment + dAdjustment;
    if (Robot.m_drive.gyroGetAngle() < 10 + desiredDegree && Robot.m_drive.gyroGetAngle() > desiredDegree - 10) {
			if (pid == false) {
				iAdjustment = 0;
				pid = true;
			}
		} else {
			pid = false;
		}

		if (pid == true) {
			if (pidAdjustment > .3) {
				speed = .3;
			} else if (pidAdjustment < -.3) {
				speed = -.3;
			} else {
				speed = pidAdjustment;
			}
		} else {
			if (error > 0) {
				speed = motorSpeed;
			} else {
				speed = -motorSpeed;
			}
		}
		Robot.m_drive.tankDrive(speed, -speed);

		if (Robot.m_drive.gyroGetAngle() < 2 + desiredDegree && Robot.m_drive.gyroGetAngle() > desiredDegree - 2) {
			n++;
			i++;
		} else {
			n = 0;
		}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.m_drive.gyroGetAngle() < 2 + desiredDegree && Robot.m_drive.gyroGetAngle() > desiredDegree - 2 && n >5);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    pid = false;
    Robot.m_drive.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
