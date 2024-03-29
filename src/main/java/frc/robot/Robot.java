// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.util.concurrent.PriorityBlockingQueue;

import javax.lang.model.util.ElementScanner6;
import javax.sound.sampled.AudioFileFormat;
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.TargetDataLine;
import javax.sound.sampled.UnsupportedAudioFileException;
import javax.sound.sampled.AudioFileFormat.Type;
import javax.sound.sampled.spi.AudioFileReader;
import javax.swing.event.MenuDragMouseEvent;

import com.fasterxml.jackson.databind.module.SimpleKeyDeserializers;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
//! IMPORTANT: IMPORT WPILIBJ2 (>>>>2<<<<) YES WITH A 2 NOT ONE BUT 2 FOR COMMAND
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drive;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;
import frc.robot.commands.DriveStr8;
import edu.wpi.first.wpilibj.Timer;

import java.io.File; 
import java.io.IOException; 
import javax.sound.sampled.AudioFormat; 
import javax.sound.sampled.AudioInputStream; 
import javax.sound.sampled.AudioSystem; 
import javax.sound.sampled.DataLine; 
import javax.sound.sampled.FloatControl; 
import javax.sound.sampled.LineUnavailableException; 
import javax.sound.sampled.SourceDataLine; 
import javax.sound.sampled.UnsupportedAudioFileException; 
import frc.robot.commands.DriveStr8;

//im not going to organize these
//deal with it (please dont)

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

  ////private File file = new File("c:/Users/bossMaster/Pictures/VXOP4165.MP4");
  /////"C:/Users/bossMaster/Documents/GitHub/I cant keep making new projects everytime theres a new robot/src/main/java/frc/robot/thhhh.wav";

	//// constructor to initialize streams and clip

 ////"C:/Users/bossMaster/Documents/GitHub/I cant keep making new projects everytime theres a new robot/src/main/java/frc/robot/thhhh.wav");

  private double one,two,three;
  private boolean lol = true;
  private double the = 3;

  private boolean poop = false;
  private boolean poopTwo = false;

  private boolean play = false;
  private boolean stop = false;
  private boolean pause = false;

  private Drive mDrive = Drive.getInstance();
  private XboxController xbox = new XboxController(0);
  private leds mleds = leds.getInstance();

 //// after dababy deal get auto working
  //dababy sound to led will never work, not enough knowledge and/or resources
  //update: dababy works, meter works, but sound to led still no go. sorry
  private Command m_autonomousCommand;
  private SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
  private ShuffleboardTab mTab = Shuffleboard.getTab("Match");
  private ComplexWidget positionChooserTab = mTab.add("Auto Chooser", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
  
  public enum AutoPosition {
    LEFT, NOTHING
  }

  private double tan = 0;

  

  private double degrees = 0;
  private double what = 0;

  private Timer timer = new Timer();
  private Timer audioTimer = new Timer();

  ////private double audio = 1;
  private double decib = 0;

  ////private LevelMeter level = new LevelMeter();

  private boolean rightBump = false;

  private double volts = 0;

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
  public void robotInit() {
    
    //Shuffleboard.startRecording();
    Shuffleboard.selectTab("Match");
    positionChooser.addOption("Nothing", AutoPosition.NOTHING);
    positionChooser.setDefaultOption("Left Trench", AutoPosition.LEFT);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Shuffle.getInstance().update();
  }

  @Override
  public void autonomousInit() {
    Shuffleboard.addEventMarker("Auto Init", EventImportance.kNormal);
    mDrive.zeroSensors();
    switch (positionChooser.getSelected()) {
      case LEFT:
        m_autonomousCommand = new DriveStr8();
        break;
      case NOTHING:
        
        break;
    }
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    mDrive.updateDrive();
  }

  @Override
  public void teleopInit() {
    Shuffleboard.addEventMarker("Teleop Init", EventImportance.kNormal);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    mDrive.zeroSensors();
    mDrive.zeroRobotNavx();
    ////mDrive.setTurnPos(0);
    timer.reset();

    
    //// TEST AND PRAY
    //// dont actually play it
    //!abandon project

    mDrive.zeroSensors();
  }

  
  @Override
  public void teleopPeriodic() {
    mDrive.updateDrive();
 
    //////AND THIS
   //// SmartDashboard.putNumber("hope", mAudio.returnToSender());
////ALSO ADD STOP AT END IF IT WORKS (UNLIKELY)
    
  //mleds.french();
  //mleds.lilNavXTWO();

  //?test below, not above
  

  //// too tired to do this now so future me do it. create a function in the 
  /*
  //simple sound file that takes a number (timer seconds) and returns the base
  // from the byte array. since the roborio can take the file, not play it, 
  // transfer the wav file to the robiro and turn the sound into byte array.
  //then start a timer here and input that number into the function so then
  // you can get sound data and then put it here. dont actually play the sound
  // get it and shit. ok bye.
  */


  //// test this bad boy

    //TODO dababy project cancelled, see top for details


  //also fix how bright leds are in leds based on max deci and /2 shit
  
  
/*
   //mleds.rickRoss(mAudio.getDeci);
    //SmartDashboard.putNumber("vol", mAudio.getVolume());
    //SmartDashboard.putNumber("deci", mAudio.getDeci());
    //if(mAudio.getDeci() > decib)
    //{
    //  decib = mAudio.getDeci();
    //}
    //SmartDashboard.putNumber("highest", decib);
*/





  //!mleds.voltage(volts);
    /*
    //if(xbox.getAButton())
    //{
    //  mDrive.setActualTurnPos(3000);
    //}
    //else if(xbox.getXButton())
    //{
    //  mDrive.setActualTurnPos(1000);
    //}
    //else
    //{
    //  mDrive.setTurnPercent(0,0,0,0);
    //}
    */
    
      
      one = (xbox.getRawAxis(1));// - DRIVE.deadband) / (1 - DRIVE.deadband);
      two = (xbox.getRawAxis(0));// - DRIVE.deadband) / (1 - DRIVE.deadband);
      three = (xbox.getRawAxis(5));// - TURN.deadband) / (1 - TURN.deadband);
      
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
      
      //// find out xbox axis shit, and pray to god
    if(poopTwo == false)
    {
      if(one != 0 || two != 0 || three != 0)
      {
        mDrive.strafeRotate(one/the,two/the,-three/the);
      }
      else if(xbox.getYButton())
      {
        mDrive.turnEncoder(0);
      }      
      else if(xbox.getXButton())
      {
        mDrive.turnInternalEncoder(180);
      }
      else if(xbox.getAButton())
      {
        mDrive.turnInternalEncoder(0);
      }
      else if(xbox.getBButton())
      {
        mDrive.turnEncoder(180);
      }
      else if(xbox.getStartButton())
      {
        mDrive.setTurnPos(0);
      }
      else
      {
        mDrive.setTurnPercent(0,0,0,0);
        mDrive.setDrivePerent(0,0,0,0);
      }
    }











    boolean a = false;
    boolean b = false;
    boolean x = false;



    
    if(poopTwo == true)
    {
      
      if(xbox.getAButtonPressed())
      {
        if(xbox.getAButtonReleased())
        {
          mDrive.playSong();
          stop = false;
          play = true;
        }
      }
      //TODO test toggle not work
      /*
// Run this only once to initialize the variables
boolean beltStatus = false;
boolean previousButton = false;
boolean currentButton = false;
// Run the following code continuously
previousButton = currentButton
currentButton = stick.getRawButton(3);

if (currentButton && !previousButton) 
{
	beltStatus = beltStatus ? false : true; 
}

conveyorMotor.set((double)(beltStatus ? 1 : 0));

credit = artdutra04 (chief delphi)
      */
      //TODO test this toggle ^^^^

      if(xbox.getBButtonPressed())
      {
        if(xbox.getBButtonReleased())
        {
          mDrive.pauseSong();
          stop = true;
          play = false;
        }
      }
      if(xbox.getXButtonPressed())
      {
        if(xbox.getXButtonReleased())
        {
          mDrive.stopSong();
          stop = true;
          play = false;
        }
      }
      if(stop && !play)
      {
        //mDrive.setTurnPercent(0,0,0,0);
        //mDrive.setDrivePerent(0,0,0,0);
      }
      
    }


      if(xbox.getBumperPressed(Hand.kLeft))
      {
        if(xbox.getBumperReleased(Hand.kLeft))
        {
          poop = !poop;
        }
      }
      if(xbox.getBumperPressed(Hand.kRight))
      {
        if(xbox.getBumperReleased(Hand.kRight))
        {
          poopTwo = !poopTwo;
        }
      }
      


      if(poop)
      {
        the = 1;
      }
      else
      {
        the = 3;
      }

      ////mDrive.forwardStrafe(one, two);
      //// test this first to see if it works, then test straferotate and diagnose problemo
      //// also tune pid, needs to happen. tune with stephan, your tuning method is ass, provides no results

      

      ////       god didnt heed my call
      //gods service provider is ass


//we've been trying to reach you regarding your robot's extended warranty

    mDrive.updateDrive();
    /*
    //y / x, opp / adj
    //but arctan uses x,y param (respectivley)
    //but tan gets other hyp, not hyp i want
    // so its y / x
    */
    tan = Math.atan2(xbox.getRawAxis(0),xbox.getRawAxis(1));    
    ////SmartDashboard.putNumber("Tan", tan);
    degrees = MkUtil.degreesToNative(tan, TURN.greerRatio);
    /*
    //SmartDashboard.putNumber("deg", degrees);

    //SmartDashboard.putNumber("one", one);
    //SmartDashboard.putNumber("two", two);
    */
    volts = RobotController.getBatteryVoltage();
    SmartDashboard.putBoolean("p", poopTwo);
  }

  @Override
  public void disabledInit() {
    ////mDrive.setActualTurnPos(0);

  }

  @Override
  public void disabledPeriodic() {
    
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  /*public boolean bringTheFightToThem()
  {
     return Math.abs(xbox.getTriggerAxis(Hand.kLeft)) > 0.8;
  }*/

 
 
}
