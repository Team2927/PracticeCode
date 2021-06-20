/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.DriveGearbox;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick joystick1;
  public JoystickButton triggered;
  
  public OI(){
    joystick1 = new Joystick(0);

    triggered = new JoystickButton(joystick1, 0);
    triggered.whileHeld(new DriveGearbox(.2));
    triggered.whenReleased(new DriveGearbox(0.0));
  }

  public Joystick getJoystick1(){
    return joystick1;
  }

  public double getJoystick1X(){
    if(Math.abs(joystick1.getX())>RobotMap.sensitivity){
      return joystick1.getX();
    } else {
      return 0.0;
    }
  }

  public double getJoystick1Y(){
    if(Math.abs(joystick1.getY())>RobotMap.sensitivity){
      return joystick1.getY();
    }else {
      return 0.0;
    }
  }
}
