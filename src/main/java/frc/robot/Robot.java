/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class Robot extends TimedRobot {

  /**
   * robot
   */

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
  }
  
  /**
   * teleop
   */

  @Override
  public void teleopInit() {
    Teleop.getInstance().init();
  }

  @Override
  public void teleopPeriodic() {
    Teleop.getInstance().loop();
  }
  
  /**
   * auton
   */

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  /**
   * test
   */
  
  TalonSRX[] talons = new TalonSRX[8];
  XboxController c = new XboxController(0);

  @Override
  public void testInit() {
    for(int i = 0; i < 8; i++) {
      talons[i]  = new TalonSRX(i); 
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if(c.getAButton()) {
      talons[0].set(ControlMode.PercentOutput, 1);
      return;
    }
    if(c.getAButtonReleased()) {
      talons[0].set(ControlMode.PercentOutput, 0);
      return;
    }
    if(c.getBButton()) {
      talons[1].set(ControlMode.PercentOutput, 1);
      return;
    }
    if(c.getBButtonReleased()) {
      talons[1].set(ControlMode.PercentOutput, 0);
      return;
    }
    if(c.getXButton()) {
      talons[2].set(ControlMode.PercentOutput, 1);
      return;
    }
    if(c.getXButtonReleased()) {
      talons[2].set(ControlMode.PercentOutput, 0);
      return;
    }
    if(c.getYButton()) {
      talons[3].set(ControlMode.PercentOutput, 1);
      return;
    }
    if(c.getYButtonReleased()) {
      talons[3].set(ControlMode.PercentOutput, 0);
      return;
    }
    talons[4].set(ControlMode.PercentOutput, c.getY(Hand.kLeft));
    talons[5].set(ControlMode.PercentOutput, c.getY(Hand.kRight));
    talons[6].set(ControlMode.PercentOutput, c.getTriggerAxis(Hand.kLeft));
    talons[7].set(ControlMode.PercentOutput, c.getTriggerAxis(Hand.kRight));
  }
}
