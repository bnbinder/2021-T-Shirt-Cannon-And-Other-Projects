// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import javax.lang.model.util.ElementScanner6;
import javax.swing.event.MenuDragMouseEvent;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;
import edu.wpi.first.wpilibj.Timer;
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

  private double one,two,three;
  private boolean lol = true;
  private double the = 3;
  private boolean poop = false;

  private Drive mDrive = Drive.getInstance();
  private XboxController xbox = new XboxController(0);
  private leds mleds = leds.getInstance();

  private double tan = 0;
  private double degrees = 0;
  private double what = 0;

  private Timer timer = new Timer();

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
    //mDrive.zeroSensors();
    mDrive.zeroRobotNavx();
    //mDrive.setTurnPos(0);
    timer.reset();
  }

  @Override
  public void teleopPeriodic() {
  //mleds.french();
  mleds.lilNavX();
  /*
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
    */
      one = (xbox.getRawAxis(1) - DRIVE.deadband) / (1 - DRIVE.deadband);
      two = (xbox.getRawAxis(0) - DRIVE.deadband) / (1 - DRIVE.deadband);
      three = (xbox.getRawAxis(5) - TURN.deadband) / (1 - TURN.deadband);
      
      if(Math.abs(xbox.getRawAxis(1)) < 0.1)
      {
        one = 0;
      }
      if(Math.abs(xbox.getRawAxis(0)) < 0.1)
      {
        two = 0;
      }
      if(Math.abs(xbox.getRawAxis(5)) < 0.1)
      {
        three = 0;
      }
      
      //TODO find out xbox axis shit, and pray to god
      if(one != 0 || two != 0 || three != 0)
      {
        mDrive.strafeRotate(-one/the,two/the,three/the);
      }
      else if(xbox.getYButton())
      {
        mDrive.turnZero();
      }      
      else if(xbox.getXButton())
      {
        mDrive.turnNine();
      }
      else if(xbox.getAButton())
      {
        mDrive.turnEight();
      }
      else if(xbox.getBButton())
      {
        mDrive.turnSeventy();
      }
      else if(Math.abs(xbox.getTriggerAxis(Hand.kLeft)) > 0)
      {
        mDrive.turnEncoderZero();
      }
      else if(xbox.getStartButton())
      {
        mDrive.setTurnPos(0);
      }
      else if(xbox.getBumperPressed(Hand.kLeft))
      {
        if(xbox.getBumperReleased(Hand.kLeft))
        {
          poop = !poop;
        }
      }
      else
      {
        mDrive.setTurnPercent(0,0,0,0);
        mDrive.setDrivePerent(0,0,0,0);
      }
      if(poop)
      {
        the = 1;
      }
      else
      {
        the = 3;
      }
      

      //mDrive.forwardStrafe(one, two);
      //TODO test this first to see if it works, then test straferotate and diagnose problemo
      //TODO also tune pid, needs to happen. tune with stephan, your tuning method is ass, provides no results

      

      //!       god didnt heed my call


//we've been trying to reach you regarding your robot's extended warranty



    mDrive.updateDrive();
    //y / x, opp / adj
    //but arctan uses x,y param (respectivley)
    //but tan gets other hyp, not hyp i want
    // so its y / x
    tan = Math.atan2(xbox.getRawAxis(0),xbox.getRawAxis(1));    
    SmartDashboard.putNumber("Tan", tan);
    degrees = MkUtil.degreesToNative(tan, TURN.greerRatio);
    SmartDashboard.putNumber("deg", degrees);

    SmartDashboard.putNumber("one", one);
    SmartDashboard.putNumber("two", two);
    SmartDashboard.putNumber("three", three);
  }

  @Override
  public void disabledInit() {
    //mDrive.setActualTurnPos(0);
  }

  @Override
  public void disabledPeriodic() {
    
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
