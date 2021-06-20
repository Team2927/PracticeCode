/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static DifferentialDrive drive;
  public static Talon left1;
  public static Talon right1;
  public static VictorSP gearboxMotor;
  
  public static ADXRS450_Gyro gyro;

  public static Encoder leftE;
  public static Encoder rightE;

  public static final double sensitivity = 0.075;
  public static final double encoderTicksToInchesConversion = 520/(4*Math.PI); //~41.380

  public static final double pStraightConstant = 0.45;
  public static final double iStraightConstant = 0.015;
  public static final double dStraightConstant = 0.011;
  public static final double gainStraightConstant = 0.03;

  public static final double pTurnConstant = .5;
  public static final double iTurnConstant = .03;
  public static final double dTurnConstant = .5;
  public static final double gainTurnConstant = .1;
  
  public static void init(){

    left1 = new Talon(1);
    right1 = new Talon(0);
    gearboxMotor = new VictorSP(2);
    leftE = new Encoder(0, 1, false);
    rightE = new Encoder(2, 3, true);
    gyro = new ADXRS450_Gyro();

    right1.setInverted(true);

    drive = new DifferentialDrive(left1, right1);
  }
}
