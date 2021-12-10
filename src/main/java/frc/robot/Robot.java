// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;
import javax.swing.event.MenuDragMouseEvent;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive;
import frc.robot.Constants.TURN;
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

  private double tan = 0;
  private double degrees = 0;
  private double what = 0;
// Locations for the swerve drive modules relative to the robot center.
/*
Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

// Creating my kinematics object using the module locations
SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);

// Creating my odometry object from the kinematics object. Here,
// our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing forward.
SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
mDrive.navXshit(), new Pose2d(0, 0, new Rotation2d()));
*/


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
    //y / x, opp / adj
    tan = Math.atan2(xbox.getRawAxis(0),xbox.getRawAxis(1));    
    SmartDashboard.putNumber("Tan", tan);
    degrees = MkUtil.degreesToNative(tan, TURN.greerRatio);
    SmartDashboard.putNumber("deg", degrees);
    
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
