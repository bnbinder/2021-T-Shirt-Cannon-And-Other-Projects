// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  private Drive mDrive = Drive.getInstance();
  private XboxController xbox = new XboxController(0);

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    mDrive.setActualTurnPos(0);
  }

  @Override
  public void teleopPeriodic() {
    
    
    if(xbox.getAButton())
    {
      mDrive.setActualTurnPos(3000);
    }
    else if(xbox.getXButton())
    {
      mDrive.setActualTurnPos(1000);
    }
    else
    {
      mDrive.setTurnPercent(0,0,0,0);
    }

    if(xbox.getBButton())
    {
      mDrive.setDrivePerent(.1, .1, .1, .1);
    }
    else
    {
      mDrive.setDrivePerent(0, 0, 0, 0);
    }
    
    mDrive.updateDrive();
  }

  @Override
  public void disabledInit() {
    mDrive.setActualTurnPos(0);
  }

  @Override
  public void disabledPeriodic() {
    mDrive.setActualTurnPos(0);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
