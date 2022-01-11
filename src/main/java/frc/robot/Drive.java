// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collection;

import javax.swing.plaf.multi.MultiDesktopIconUI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;
import frc.robot.MkUtil;


/** Add your docs here. */
public class Drive {
    private final AHRS navX = new AHRS();
    
    private final TalonFX topDriveRight = new TalonFX(DRIVE.topDriveRightCANID);
    private final TalonFX topTurnRight = new TalonFX(TURN.topTurnRightCANID);
    
    private final TalonFX bottomDriveRight = new TalonFX(DRIVE.bottomDriveRightCANID);
    private final TalonFX bottomTurnRight = new TalonFX(TURN.bottomTurnRightCANID);
   
    private final TalonFX topDriveLeft = new TalonFX(DRIVE.topDriveLeftCANID);
    private final TalonFX topTurnLeft = new TalonFX(TURN.topTurnLeftCANID);
    
    private final TalonFX bottomDriveLeft = new TalonFX(DRIVE.bottomDriveLeftCANID);
    private final TalonFX bottomTurnLeft = new TalonFX(TURN.bottomTurnLeftCANID);

    private final CANCoder topTurnLeftEncoder = new CANCoder(TURN.topTurnLeftCANCoderCANID);
    private final CANCoder topTurnRightEncoder = new CANCoder(TURN.topTurnRightCANCoderCANID);
    private final CANCoder bottomTurnLeftEncoder = new CANCoder(TURN.bottomTurnLeftCANCoderCANID);
    private final CANCoder bottomTurnRightEncoder = new CANCoder(TURN.bottomTurnRightCANCoderCANID);

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(TURN.maxVel, TURN.maxAccel);
    //only use this for autonomous, you moron

    private final PIDController turning = new PIDController(TURN.turnKP, TURN.turnKI, TURN.turnKD);

    private final PIDController turningTopLeft = new PIDController(TURN.turnKP, TURN.turnKI, TURN.turnKD);
    private final PIDController turningTopRight = new PIDController(TURN.turnKP, TURN.turnKI, TURN.turnKD);
    private final PIDController turningBotLeft = new PIDController(TURN.turnKP, TURN.turnKI, TURN.turnKD);
    private final PIDController turningBotRight = new PIDController(TURN.turnKP, TURN.turnKI, TURN.turnKD);

    private final PIDController driveTopLeft = new PIDController(DRIVE.driveKPCalc, DRIVE.driveKICalc, DRIVE.driveKDCalc);
    private final PIDController driveTopRight = new PIDController(DRIVE.driveKPCalc, DRIVE.driveKICalc, DRIVE.driveKDCalc);
    private final PIDController driveBotLeft = new PIDController(DRIVE.driveKPCalc, DRIVE.driveKICalc, DRIVE.driveKDCalc);
    private final PIDController driveBotRight = new PIDController(DRIVE.driveKPCalc, DRIVE.driveKICalc, DRIVE.driveKDCalc);
    //TODO you dont need multiple controllers, calculating only takes the current state and goal. they arent wired into them, only inputted, you moron
    //!do it at guerin to see if it fucks things up tho, saftey first

    private final PIDController topLeftTurningEncoder = new PIDController(TURN.turnEncoderKP, TURN.turnEncoderKI, TURN.turnEncoderKD);
    private final PIDController topRightTurningEncoder = new PIDController(TURN.turnEncoderKP, TURN.turnEncoderKI, TURN.turnEncoderKD);
    private final PIDController bottomLeftTurningEncoder = new PIDController(TURN.turnEncoderKP, TURN.turnEncoderKI, TURN.turnEncoderKD);
    private final PIDController bottomRightTurningEncoder = new PIDController(TURN.turnEncoderKP, TURN.turnEncoderKI, TURN.turnEncoderKD);

    private double offsetTopLeftCANCoder;// = topTurnLeftEncoder.getAbsolutePosition() - -107.050781;//78.75;
    private double offsetTopRightCANCoder;// = topTurnRightEncoder.getAbsolutePosition() - -67.3242187;//115.224;
    private double offsetBottomLeftCANCoder;// = bottomTurnLeftEncoder.getAbsolutePosition() - -63.89648437; //117.0703125;//121.289;
    private double offsetBottomRightCANCoder;// = bottomTurnRightEncoder.getAbsolutePosition() - 134.1210937;//320.361;

    private double distance;
    
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
    //relative to center of robot, in meters (?)

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
     m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

        

    // Creating my odometry object from the kinematics object. Here,
    // our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing forward.
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, 
    Rotation2d.fromDegrees(navXshit()), new Pose2d(5.0, 13.5, new Rotation2d()));

    //kinematics, gyro, x, y, 0deg(forward)


    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.

                                    //forward, side, rotate
   //ChassisSpeeds speeds = new ChassisSpeeds(1, 3, 1.5);

    
    // The desired field relative speed here is 2 meters per second
    // toward the opponent's alliance station wall, and 2 meters per
    // second toward the left field boundary. The desired rotation
    // is a quarter of a rotation per second counterclockwise. The current
    // robot angle is 45 degrees.
   /* 
    ChassisSpeeds speedsField = ChassisSpeeds.fromFieldRelativeSpeeds(
        2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(navXshit()));
        //forward, side, rotate, angle
    */



    //Pose2d m_pose;

    ChassisSpeeds speeds;
    SwerveModuleState[] moduleStates;
    SwerveModuleState frontLeft, frontRight, bottomLeft, bottomRight;
   
    private double leftTopPosNative, leftBottomPosNative, 
                   rightTopPosNative, rightBottomPosNative,
                   
                   leftTopPosInch, leftBottomPosInch,
                   rightTopPosInch, rightBottomPosInch,

                   leftTopPosMeters, leftBottomPosMeters,
                   rightTopPosMeters, rightBottomPosMeters,
                   
                   leftTopVelInch, leftBottomVelInch,
                   rightTopVelInch, rightBottomVelInch,

                   leftTopVelNative, leftBottomVelNative,
                   rightTopVelNative, rightBottomVelNative,
                   
                   leftTopVelMeters, leftBottomVelMeters,
                   rightTopVelMeters, rightBottomVelMeters,
                   
                   avgVelInches, avgDistInches,

                   leftTopDeg, leftBottomDeg,
                   rightTopDeg, rightBottomDeg,

                   leftTopOutput, leftBottomOutput,
                   rightTopOutput, rightBottomOutput;

    SwerveModuleState frontLeftState, frontRightState, backLeftState, backRightState;
    ChassisSpeeds chassisSpeeds;
    double forward, sideways, angular;
    //public Collection<TalonFX> collect;
    public Orchestra dababy;
    public String path = "chirp.chrp";//"C:/Users/bossMaster/Desktop/h.wav";
    
    private Drive()
    { 
        //// set integrated sensor for all thingies 
        topDriveLeft.configFactoryDefault();
        bottomDriveLeft.configFactoryDefault();
        topDriveRight.configFactoryDefault();
        bottomDriveRight.configFactoryDefault();

        topTurnLeft.configFactoryDefault();
        bottomTurnLeft.configFactoryDefault();
        topTurnRight.configFactoryDefault();
        bottomTurnRight.configFactoryDefault();
    

        topDriveLeft.setNeutralMode(NeutralMode.Brake);
        topDriveRight.setNeutralMode(NeutralMode.Brake);
        bottomDriveLeft.setNeutralMode(NeutralMode.Brake);
        bottomDriveRight.setNeutralMode(NeutralMode.Brake);
        
        topTurnLeft.setNeutralMode(NeutralMode.Brake);
        topTurnRight.setNeutralMode(NeutralMode.Brake);
        bottomTurnLeft.setNeutralMode(NeutralMode.Brake);
        bottomTurnRight.setNeutralMode(NeutralMode.Brake);


        topTurnLeft.config_kP(0, TURN.turnKP);
        topTurnLeft.config_kI(0, TURN.turnKI);
        topTurnLeft.config_kD(0, TURN.turnKD);
        topTurnLeft.config_kF(0, TURN.turnKF);

        topTurnRight.config_kP(0, TURN.turnKP);
        topTurnRight.config_kI(0, TURN.turnKI);
        topTurnRight.config_kD(0, TURN.turnKD);
        topTurnRight.config_kF(0, TURN.turnKF);

        bottomTurnLeft.config_kP(0, TURN.turnKP);
        bottomTurnLeft.config_kI(0, TURN.turnKI);
        bottomTurnLeft.config_kD(0, TURN.turnKD);
        bottomTurnLeft.config_kF(0, TURN.turnKF);

        bottomTurnRight.config_kP(0, TURN.turnKP);
        bottomTurnRight.config_kI(0, TURN.turnKI);
        bottomTurnRight.config_kD(0, TURN.turnKD);
        bottomTurnRight.config_kF(0, TURN.turnKF);


        topDriveLeft.config_kP(0, DRIVE.driveKP);
        topDriveLeft.config_kI(0, DRIVE.driveKI);
        topDriveLeft.config_kD(0, DRIVE.driveKD);
        topDriveLeft.config_kF(0, DRIVE.driveKF);

        topDriveRight.config_kP(0, DRIVE.driveKP);
        topDriveRight.config_kI(0, DRIVE.driveKI);
        topDriveRight.config_kD(0, DRIVE.driveKD);
        topDriveRight.config_kF(0, DRIVE.driveKF);

        bottomDriveLeft.config_kP(0, DRIVE.driveKP);
        bottomDriveLeft.config_kI(0, DRIVE.driveKI);
        bottomDriveLeft.config_kD(0, DRIVE.driveKD);
        bottomDriveLeft.config_kF(0, DRIVE.driveKF);

        bottomDriveRight.config_kP(0, DRIVE.driveKP);
        bottomDriveRight.config_kI(0, DRIVE.driveKI);
        bottomDriveRight.config_kD(0, DRIVE.driveKD);
        bottomDriveRight.config_kF(0, DRIVE.driveKF);


        topDriveLeft.setInverted(false);
        topDriveRight.setInverted(false);
        bottomDriveLeft.setInverted(false);
        bottomDriveRight.setInverted(false);

        topTurnLeft.setInverted(false);
        topTurnRight.setInverted(false);
        bottomTurnLeft.setInverted(false);
        bottomTurnRight.setInverted(false);

        topDriveLeft.configMotionSCurveStrength(6);
        topDriveRight.configMotionSCurveStrength(6);
        bottomDriveLeft.configMotionSCurveStrength(6);
        bottomDriveRight.configMotionSCurveStrength(6);
        //TODO see how scurve changes accuracy
        
        topDriveLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
        topDriveLeft.configVelocityMeasurementWindow(16);

        topDriveRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
        topDriveRight.configVelocityMeasurementWindow(16);

        bottomDriveLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
        bottomDriveLeft.configVelocityMeasurementWindow(16);

        bottomDriveRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
        bottomDriveRight.configVelocityMeasurementWindow(16);


        topTurnLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
        topTurnLeft.configVelocityMeasurementWindow(16);

        topTurnRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
        topTurnRight.configVelocityMeasurementWindow(16);

        bottomTurnLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
        bottomTurnLeft.configVelocityMeasurementWindow(16);

        bottomTurnRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
        bottomTurnRight.configVelocityMeasurementWindow(16);

        
        topDriveLeft.configVoltageCompSaturation(DRIVE.voltComp);
        topDriveLeft.enableVoltageCompensation(true);
        
        topTurnLeft.configVoltageCompSaturation(DRIVE.voltComp);
        topTurnLeft.enableVoltageCompensation(true);

        topDriveRight.configVoltageCompSaturation(DRIVE.voltComp);
        topDriveRight.enableVoltageCompensation(true);

        topTurnRight.configVoltageCompSaturation(DRIVE.voltComp);
        topTurnRight.enableVoltageCompensation(true);

        bottomDriveLeft.configVoltageCompSaturation(DRIVE.voltComp);
        bottomDriveLeft.enableVoltageCompensation(true);

        bottomTurnLeft.configVoltageCompSaturation(DRIVE.voltComp);
        bottomTurnLeft.enableVoltageCompensation(true);

        bottomDriveRight.configVoltageCompSaturation(DRIVE.voltComp);
        bottomDriveRight.enableVoltageCompensation(true);

        bottomTurnRight.configVoltageCompSaturation(DRIVE.voltComp);
        bottomTurnRight.enableVoltageCompensation(true);


        topDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        topDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        topTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        topTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        topDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        topDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        topTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        topTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        bottomDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        bottomDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        bottomTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        bottomTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        bottomDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        bottomDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);

        bottomTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10);
        bottomTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10);


        topDriveLeft.configPeakOutputForward(DRIVE.peakOutputForward);
        topDriveLeft.configPeakOutputReverse(DRIVE.peakOutputReverse);

        topDriveRight.configPeakOutputForward(DRIVE.peakOutputForward);
        topDriveRight.configPeakOutputReverse(DRIVE.peakOutputReverse);

        bottomDriveLeft.configPeakOutputForward(DRIVE.peakOutputForward);
        bottomDriveLeft.configPeakOutputReverse(DRIVE.peakOutputReverse);

        bottomDriveRight.configPeakOutputForward(DRIVE.peakOutputForward);
        bottomDriveRight.configPeakOutputReverse(DRIVE.peakOutputReverse);


        topDriveLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        topDriveRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomDriveLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomDriveRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        topTurnLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        topTurnRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomTurnLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        bottomTurnRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

/*
        topTurnLeft.configPeakOutputForward(DRIVE.peakOutputForward);
        topTurnLeft.configPeakOutputReverse(DRIVE.peakOutputReverse);

        topTurnRight.configPeakOutputForward(DRIVE.peakOutputForward);
        topTurnRight.configPeakOutputReverse(DRIVE.peakOutputReverse);

        bottomTurnLeft.configPeakOutputForward(DRIVE.peakOutputForward);
        bottomTurnLeft.configPeakOutputReverse(DRIVE.peakOutputReverse);

        bottomTurnRight.configPeakOutputForward(DRIVE.peakOutputForward);
        bottomTurnRight.configPeakOutputReverse(DRIVE.peakOutputReverse);
*/
        //what this do?
        //topTurnRight.configAllowableClosedloopError(0, 1);

        //makes sure motors dont go above 1 (100%)?
        topTurnLeft.configClosedLoopPeakOutput(0, DRIVE.peakOutputForward);
        topTurnRight.configClosedLoopPeakOutput(0, DRIVE.peakOutputForward);
        bottomTurnLeft.configClosedLoopPeakOutput(0, DRIVE.peakOutputForward);
        bottomTurnRight.configClosedLoopPeakOutput(0, DRIVE.peakOutputForward);

        topDriveLeft.configClosedLoopPeakOutput(0, DRIVE.peakOutputForward);
        topDriveRight.configClosedLoopPeakOutput(0, DRIVE.peakOutputForward);
        bottomDriveLeft.configClosedLoopPeakOutput(0, DRIVE.peakOutputForward);
        bottomDriveRight.configClosedLoopPeakOutput(0, DRIVE.peakOutputForward);

        
        //during open loop, 0.5 seconds before motor go from 0 to selected output
        topTurnLeft.configOpenloopRamp(TURN.turnOpenRampRate);
        topTurnRight.configOpenloopRamp(TURN.turnOpenRampRate);
        bottomTurnLeft.configOpenloopRamp(TURN.turnOpenRampRate);
        bottomTurnRight.configOpenloopRamp(TURN.turnOpenRampRate);

        topDriveLeft.configOpenloopRamp(DRIVE.driveOpenRampRate);
        topDriveRight.configOpenloopRamp(DRIVE.driveOpenRampRate);
        bottomDriveLeft.configOpenloopRamp(DRIVE.driveOpenRampRate);
        bottomDriveRight.configOpenloopRamp(DRIVE.driveOpenRampRate);


        //during close loop, 0.5 seconds before motor go from 0 to selected output
        topTurnLeft.configClosedloopRamp(TURN.turnCloseRampRate);
        topTurnRight.configClosedloopRamp(TURN.turnCloseRampRate);
        bottomTurnLeft.configClosedloopRamp(TURN.turnCloseRampRate);
        bottomTurnRight.configClosedloopRamp(TURN.turnCloseRampRate);


        topDriveLeft.configClosedloopRamp(DRIVE.driveCloseRampRate);
        topDriveRight.configClosedloopRamp(DRIVE.driveCloseRampRate);
        bottomDriveLeft.configClosedloopRamp(DRIVE.driveCloseRampRate);
        bottomDriveRight.configClosedloopRamp(DRIVE.driveCloseRampRate);


        topTurnLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        topTurnRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        bottomTurnLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        bottomTurnRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    //    topTurnLeftEncoder.configMagnetOffset(-offsetTopLeftCANCoder/*251.2*/);
  //      topTurnRightEncoder.configMagnetOffset(-offsetTopRightCANCoder/*286.7*/);
//     bottomTurnLeftEncoder.configMagnetOffset(-offsetBottomLeftCANCoder/*298.3*/);
  //      bottomTurnRightEncoder.configMagnetOffset(-offsetBottomRightCANCoder/*130.4*/);
   offsetTopLeftCANCoder = topTurnLeftEncoder.getAbsolutePosition() - -106.787109375;//-107.050781;//78.75;
 offsetTopRightCANCoder = topTurnRightEncoder.getAbsolutePosition() - -71.103515625;//-67.3242187;//115.224;
  offsetBottomLeftCANCoder = bottomTurnLeftEncoder.getAbsolutePosition() - -66.708984375;//-63.89648437; //117.0703125;//121.289;
offsetBottomRightCANCoder = bottomTurnRightEncoder.getAbsolutePosition() - 132.9785156;//134.1210937;//320.361;

        topTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(offsetTopLeftCANCoder, TURN.greerRatio));
        topTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(offsetTopRightCANCoder, TURN.greerRatio));
        bottomTurnLeft.setSelectedSensorPosition(MkUtil.degreesToNative(offsetBottomLeftCANCoder, TURN.greerRatio));
        bottomTurnRight.setSelectedSensorPosition(MkUtil.degreesToNative(offsetBottomRightCANCoder, TURN.greerRatio));

        

        dababy = new Orchestra();
        TalonFX [] _fxes = {new TalonFX(DRIVE.topDriveLeftCANID), new TalonFX(DRIVE.topDriveRightCANID), new TalonFX(DRIVE.bottomDriveLeftCANID), new TalonFX(DRIVE.bottomDriveRightCANID), new TalonFX(TURN.topTurnLeftCANID), new TalonFX(TURN.topTurnRightCANID), new TalonFX(TURN.bottomTurnLeftCANID), new TalonFX(TURN.bottomTurnRightCANID)};
        ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
        for(int i = 0; i < _fxes.length; ++i)
        {
            _instruments.add(_fxes[i]);
        }
        dababy = new Orchestra(_instruments);
        dababy.loadMusic(path);
       
    }

    public static Drive getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateDrive()
    {
        
        leftTopVelNative = topDriveLeft.getSelectedSensorVelocity();
        rightTopVelNative = topDriveRight.getSelectedSensorVelocity();
        leftBottomVelNative = bottomDriveLeft.getSelectedSensorVelocity();
        rightBottomVelNative = bottomDriveRight.getSelectedSensorVelocity();

        leftTopPosNative = topDriveLeft.getSelectedSensorPosition();
        rightTopPosNative = topDriveRight.getSelectedSensorPosition();
        leftBottomPosNative = bottomDriveLeft.getSelectedSensorPosition();
        rightBottomPosNative = bottomDriveRight.getSelectedSensorPosition();

        leftTopVelInch = MkUtil.nativePer100MstoInchesPerSec(leftTopVelNative);
        rightTopVelInch = MkUtil.nativePer100MstoInchesPerSec(rightTopVelNative);
        leftBottomVelInch = MkUtil.nativePer100MstoInchesPerSec(leftBottomVelNative);
        rightBottomVelInch = MkUtil.nativePer100MstoInchesPerSec(rightBottomVelNative);

        leftTopPosInch = MkUtil.nativeToInches(leftTopPosNative);
        rightTopPosInch = MkUtil.nativeToInches(rightTopPosNative);
        leftBottomPosInch = MkUtil.nativeToInches(leftBottomPosNative);
        rightBottomPosInch = MkUtil.nativeToInches(rightBottomPosNative);
        
        leftTopVelMeters = MkUtil.nativePer100MsToMetersPerSec(leftTopVelNative);
        rightTopVelMeters = MkUtil.nativePer100MsToMetersPerSec(rightTopVelNative);
        leftBottomVelMeters = MkUtil.nativePer100MsToMetersPerSec(leftBottomVelNative);
        rightBottomVelMeters = MkUtil.nativePer100MsToMetersPerSec(rightBottomVelNative);

        leftTopPosMeters = MkUtil.nativeToMeters(leftTopPosNative);
        rightTopPosMeters = MkUtil.nativeToMeters(rightTopPosNative);
        leftBottomPosMeters = MkUtil.nativeToMeters(leftBottomPosNative);
        rightBottomPosMeters = MkUtil.nativeToMeters(rightBottomPosNative);

        avgDistInches = (leftTopPosInch + rightTopPosInch + leftBottomPosInch + rightBottomPosInch) /4.0;
        avgVelInches = (leftTopVelInch + rightTopVelInch + leftBottomVelInch + rightBottomVelInch) / 4.0;
        //TODO for avg dist need to figure out how to do avg dist since four motors four wheels not tank drive, same with avg vel
        //!this is only for if i have the brains to implement swerve unicorn auto drive, doesnt affect reg tank drive activities

        leftTopDeg = MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio);
        rightTopDeg = MkUtil.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), TURN.greerRatio);
        leftBottomDeg = MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio);
        rightBottomDeg = MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio);
        ////invalid, go above 360, use the cancoders
        //! or just use % you moron

        frontLeftState = new SwerveModuleState(leftTopVelMeters, Rotation2d.fromDegrees(leftTopDeg));
        frontRightState = new SwerveModuleState(rightTopVelMeters, Rotation2d.fromDegrees(rightTopDeg));
        backLeftState = new SwerveModuleState(leftBottomVelMeters, Rotation2d.fromDegrees(leftBottomDeg));
        backRightState = new SwerveModuleState(rightBottomVelMeters, Rotation2d.fromDegrees(rightBottomDeg));

        // Convert to chassis speeds
        chassisSpeeds = m_kinematics.toChassisSpeeds(
        frontLeftState, frontRightState, backLeftState, backRightState);

        // Getting individual speeds
        forward = chassisSpeeds.vxMetersPerSecond;
        sideways = chassisSpeeds.vyMetersPerSecond;
        angular = chassisSpeeds.omegaRadiansPerSecond;

        SmartDashboard.putNumber("forward", forward);
        SmartDashboard.putNumber("sideways", sideways);
        SmartDashboard.putNumber("angular", angular);

        //speeds = new ChassisSpeeds(forward, sideways, angular);

        //moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        
        //Front left module state
        //frontLeft = moduleStates[0];
    
        // Front right module state
        //frontRight = moduleStates[1];
    
        // Back left module state
        //bottomLeft = moduleStates[2];
    
        // Back right module state
        //bottomRight = moduleStates[3];

        /*
        SmartDashboard.putNumber("1top turn right", canIPutMy1);
        SmartDashboard.putNumber("2top turn left", canIPutMy2);
        SmartDashboard.putNumber("3bot turn right", canIPutMy3);
        SmartDashboard.putNumber("4bot turn left", canIPutMy4);

        SmartDashboard.putNumber("mr", ballsInYoJaw);
        */

        //degrees = MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio);
        SmartDashboard.putNumber("topl", topTurnLeft.getSelectedSensorPosition());//, TURN.greerRatio));
        SmartDashboard.putNumber("topr", topTurnRight.getSelectedSensorPosition());//, TURN.greerRatio));
        SmartDashboard.putNumber("botl", bottomTurnLeft.getSelectedSensorPosition());//, TURN.greerRatio));
        SmartDashboard.putNumber("botr", bottomTurnRight.getSelectedSensorPosition());//, TURN.greerRatio));
        //(frontLeft,frontRight,bottomLeft,bottomRight);
        
        //frontLeft = new SwerveModuleState(leftTopVelMeters, Rotation2d.fromDegrees(MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio)));
        //frontRight = new SwerveModuleState(rightTopVelMeters, Rotation2d.fromDegrees(MkUtil.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), TURN.greerRatio)));
        //bottomLeft = new SwerveModuleState(leftBottomVelMeters, Rotation2d.fromDegrees(MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio)));
        //bottomRight = new SwerveModuleState(rightBottomVelMeters, Rotation2d.fromDegrees(MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio)));
        m_odometry.update(Rotation2d.fromDegrees(navXshit()), frontLeftState,frontRightState,backLeftState,backRightState);
        

       SmartDashboard.putNumber("x", m_odometry.getPoseMeters().getX());
       SmartDashboard.putNumber("y", m_odometry.getPoseMeters().getY());


        SmartDashboard.putNumber("navx", navX.getYaw());

        SmartDashboard.putNumber("encoder t left", MkUtil.degreesToNative(topTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        SmartDashboard.putNumber("encoder t right", MkUtil.degreesToNative(topTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));
        SmartDashboard.putNumber("encoder b left", MkUtil.degreesToNative(bottomTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio));
        SmartDashboard.putNumber("encoder b right", MkUtil.degreesToNative(bottomTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio));

        SmartDashboard.putNumber("speeed", topDriveLeft.getSelectedSensorVelocity());


       
    }



    public void playSong()
    {
        dababy.play();
    }

    public void stopSong()
    {
        dababy.stop();
    }

    public void pauseSong()
    {
        dababy.pause();
    }




    public void setTurnPos(double position)
    {
        topTurnLeft.setSelectedSensorPosition(position);
        topTurnRight.setSelectedSensorPosition(position);
        bottomTurnLeft.setSelectedSensorPosition(position);
        bottomTurnRight.setSelectedSensorPosition(position);
    }

    public void setActualTurnPos(double position)
    {
        topTurnLeft.set(ControlMode.Position, position);
        topTurnRight.set(ControlMode.Position, position);
        bottomTurnLeft.set(ControlMode.Position, position);
        bottomTurnRight.set(ControlMode.Position, position);
    }

    public void setTurnPercent(double topleft, double topright, double botleft, double botright)
    {
        topTurnLeft.set(ControlMode.PercentOutput, topleft);
        topTurnRight.set(ControlMode.PercentOutput, topright);
        bottomTurnLeft.set(ControlMode.PercentOutput, botleft);
        bottomTurnRight.set(ControlMode.PercentOutput, botright);
    }

    public void setDrivePerent(double topLeft, double topRight, double botLeft, double botRight)
    {
        topDriveLeft.set(ControlMode.PercentOutput, topLeft);
        topDriveRight.set(ControlMode.PercentOutput, topRight);
        bottomDriveLeft.set(ControlMode.PercentOutput, botLeft);
        bottomDriveRight.set(ControlMode.PercentOutput, botRight);
    }



    public double turnCalculateTopLeft(double setpoint)
    {
        return turningTopLeft.calculate(topTurnLeft.getSelectedSensorPosition(), setpoint);
    }
    public double turnCalculateTopRight(double setpoint)
    {
        return turningTopRight.calculate(topTurnRight.getSelectedSensorPosition(), setpoint);
    }
    public double turnCalculateBotLeft(double setpoint)
    {
        return turningBotLeft.calculate(bottomTurnLeft.getSelectedSensorPosition(), setpoint);
    }
    public double turnCalculateBotRight(double setpoint)
    {
        return turningBotRight.calculate(bottomTurnRight.getSelectedSensorPosition(), setpoint);
    }



    public double driveCalculateTopLeft(double setpoint)
    {
        return driveTopLeft.calculate(topDriveLeft.getMotorOutputPercent(), setpoint);
    }
    public double driveCalculateTopRight(double setpoint)
    {
        return driveTopRight.calculate(topDriveRight.getMotorOutputPercent(), setpoint);
    }
    public double driveCalculateBotLeft(double setpoint)
    {
        return driveBotLeft.calculate(bottomDriveLeft.getMotorOutputPercent(), setpoint);
    }
    public double driveCalculateBotRight(double setpoint)
    {
        return driveBotRight.calculate(bottomDriveRight.getMotorOutputPercent(), setpoint);
    }





    public double turnCalculateTopLeftEncoder(double setpoint)
    {
        return topLeftTurningEncoder.calculate((topTurnLeft.getSelectedSensorPosition()), (MkUtil.degreesToNative(MkUtil.setDirection(topTurnLeft, setpoint, TURN.greerRatio),TURN.greerRatio)));
    }
    public double turnCalculateTopRightEncoder(double setpoint)
    {
        return topRightTurningEncoder.calculate((topTurnRight.getSelectedSensorPosition()), (MkUtil.degreesToNative(MkUtil.setDirection(topTurnRight, setpoint, TURN.greerRatio),TURN.greerRatio)));
    }
    public double turnCalculateBotLeftEncoder(double setpoint)
    {
        return bottomLeftTurningEncoder.calculate((bottomTurnLeft.getSelectedSensorPosition()), (MkUtil.degreesToNative(MkUtil.setDirection(bottomTurnLeft, setpoint, TURN.greerRatio),TURN.greerRatio)));
    }
    public double turnCalculateBotRightEncoder(double setpoint)
    {
        return bottomRightTurningEncoder.calculate((bottomTurnRight.getSelectedSensorPosition()), (MkUtil.degreesToNative(MkUtil.setDirection(bottomTurnRight, setpoint, TURN.greerRatio),TURN.greerRatio)));
    }



/*
    public double turnDegCalculate(double setpoint)
    {
        setpoint = MkUtil.degreesToNative(setpoint, TURN.greerRatio);
        return turning.calculate(topTurnLeft.getSelectedSensorPosition(), setpoint);
    }

    ////may not need these
    and not need these i didnt
    does this even make sense
*/

    public void zeroSensors() 
    {
        topDriveLeft.setSelectedSensorPosition(0);
        topDriveRight.setSelectedSensorPosition(0);
        bottomDriveLeft.setSelectedSensorPosition(0);
        bottomDriveRight.setSelectedSensorPosition(0);
    }    
    

    public void setMagicStraight(double setpoint)
    {
        zeroSensors();
        distance = setpoint;
        topDriveLeft.configMotionCruiseVelocity(DRIVE.magicVel);
        topDriveRight.configMotionCruiseVelocity(DRIVE.magicVel);
        bottomDriveLeft.configMotionCruiseVelocity(DRIVE.magicVel);
        bottomDriveRight.configMotionCruiseVelocity(DRIVE.magicVel);

        topDriveLeft.configMotionAcceleration(DRIVE.magicAccel);
        topDriveRight.configMotionAcceleration(DRIVE.magicAccel);
        bottomDriveLeft.configMotionAcceleration(DRIVE.magicAccel);
        bottomDriveRight.configMotionAcceleration(DRIVE.magicAccel);

        //zeroSensors();
    }

    public void updateMagicStraight()
    {
        topDriveLeft.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        topDriveRight.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        bottomDriveLeft.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));
        bottomDriveRight.set(ControlMode.MotionMagic, MkUtil.inchesToNative(distance));

        leftTopOutput = MkUtil.inchesToNative(distance);
        rightTopOutput = MkUtil.inchesToNative(distance);
        leftBottomOutput = MkUtil.inchesToNative(distance);
        rightBottomOutput = MkUtil.inchesToNative(distance);

        SmartDashboard.putNumber("dist", distance);
        SmartDashboard.putNumber("leftout", leftTopOutput);
    }

    public boolean isMagicStraightDone()
    {
        double err = distance - avgDistInches;
        return Math.abs(err) < 0.5 && Math.abs(avgVelInches) < 0.1;
    }

    public Rotation2d navXshit2()
    {
        return navX.getRotation2d();
    }

    public double navXshit()
    {
        return navX.getYaw();
    }

    public void inversionOne(double last, double wa, TalonFX talon)
    {
        if(Math.abs(last - wa) > 70)
        {
            talon.setInverted(!talon.getInverted());
        }
    }

    public double inversionTwo(double last, double wa, TalonFX talon)
    {
        if(talon.getInverted())
        {
            return ((wa + 360) % 360) - 180;
        }
        else
        {
            return wa;
        }
    }

    public static void inversionAwarness(TalonFX talon, double wa)
    {
    double encoderw = talon.getSelectedSensorPosition();
    double azimuthAngle = encoderw;
    double azimuthError = azimuthAngle - wa;

    if(Math.abs(azimuthError) > 90)//assuming our angles are in degrees
    {
      azimuthError = azimuthError - 180 * Math.signum(azimuthError);
      talon.setInverted(true);
    }
      else
      {
        talon.setInverted(false);
      }
    }

    public void strafeRotate(double FWD, double STR, double RCW)
    {
        /*
        double mrKrabs = Math.atan2(why,ex);

        double amongusTL = (power1 / Math.abs(power1)) * (-45);
        double amongusTR = (power1 / Math.abs(power1)) * (-135);
        double amongusBL = (power1 / Math.abs(power1)) * (-225);
        double amongusBR = (power1 / Math.abs(power1)) * (-315);

        double length1 = Math.sqrt(Math.pow(ex, 2) + Math.pow(why, 2));
        double length2 = Math.sqrt(Math.pow(power1, 2) + Math.pow(power2, 2));

        double rottenFleschTL = Math.sqrt(Math.pow(amongusTL, 2) + Math.pow(mrKrabs, 2));
        double rottenFleschTR = Math.sqrt(Math.pow(amongusTR, 2) + Math.pow(mrKrabs, 2));
        double rottenFleschBL = Math.sqrt(Math.pow(amongusBL, 2) + Math.pow(mrKrabs, 2));
        double rottenFleschBR = Math.sqrt(Math.pow(amongusBR, 2) + Math.pow(mrKrabs, 2));

        double powerBros = Math.sqrt(Math.pow(length2, 2) + Math.pow(length1, 2));

        canIPutMy1 = rottenFleschTL;
        canIPutMy2 = rottenFleschTR;
        canIPutMy3 = rottenFleschBL;
        canIPutMy4 = rottenFleschBR;
        ballsInYoJaw = powerBros;
        */
        //my version was somewhat close, gotta gimme some credit
        double nav = navXshit();
        double temp = FWD*Math.cos(Math.toRadians(nav)) + STR*Math.sin(Math.toRadians(nav));
        STR = -FWD*Math.sin(Math.toRadians(nav)) + STR*Math.cos(Math.toRadians(nav));
        FWD = temp; 

        SmartDashboard.putNumber("fwd", FWD);
        SmartDashboard.putNumber("str", STR);

        double A = STR - RCW*(Constants.L/Constants.R); //-
        double B = STR + RCW*(Constants.L/Constants.R); //+
        double C = FWD - RCW*(Constants.W/Constants.R); //-
        double D = FWD + RCW*(Constants.W/Constants.R); //+

        SmartDashboard.putNumber("a", A);
        SmartDashboard.putNumber("b", B);
        SmartDashboard.putNumber("c", C);
        SmartDashboard.putNumber("d", D);

        double ws1 = Math.sqrt(Math.pow(B,2)+Math.pow(C,2));   double wa1 = Math.atan2(B,C)*(180/Math.PI);
        double ws2 = Math.sqrt(Math.pow(B,2)+Math.pow(D,2));   double wa2 = Math.atan2(B,D)*(180/Math.PI);
        double ws3 = Math.sqrt(Math.pow(A,2)+Math.pow(D,2));   double wa3 = Math.atan2(A,D)*(180/Math.PI);
        double ws4 = Math.sqrt(Math.pow(A,2)+Math.pow(C,2));   double wa4 = Math.atan2(A,C)*(180/Math.PI);

        //// add these ifs after testing if constraints fuck things up

        //sus about this, very sus
        //nvm its good doesnt break the code
        /*
        double max=ws1; if(ws2>max)max=ws2; if(ws3>max)max=ws3; if(ws4>max)max=ws4;
        if(max>1){ws1/=max; ws2/=max; ws3/=max; ws4/=max;}
*/
        wa2 = MkUtil.setDirection(topTurnLeft, wa2, driveTopLeft);
        wa1 = MkUtil.setDirection(topTurnRight, wa1, driveTopRight);
        wa4 = MkUtil.setDirection(bottomTurnRight, wa4, driveBotRight);
        wa3 = MkUtil.setDirection(bottomTurnLeft, wa3, driveBotLeft);

        ws2 = MkUtil.isPositive(driveTopLeft.getP(), ws2);
        ws1 = MkUtil.isPositive(driveTopRight.getP(), ws1);
        ws4 = MkUtil.isPositive(driveBotRight.getP(), ws4);
        ws3 = MkUtil.isPositive(driveBotLeft.getP(), ws3);

        topDriveLeft.set(ControlMode.PercentOutput, ws2);
        topDriveRight.set(ControlMode.PercentOutput, ws1);
        bottomDriveRight.set(ControlMode.PercentOutput, ws4);
        bottomDriveLeft.set(ControlMode.PercentOutput, ws3);
/*
        inversionOne(lastwa1,wa1,topDriveRight);
        inversionOne(lastwa2,wa2,topDriveLeft);
        inversionOne(lastwa3,wa3,bottomDriveLeft);
        inversionOne(lastwa4,wa4,bottomDriveRight);
        lastwa1 = wa1;
        lastwa2 = wa2;
        lastwa3 = wa3;
        lastwa4 = wa4;
        inversionTwo(lastwa1,wa1,topDriveRight);
        inversionTwo(lastwa2,wa2,topDriveLeft);
        inversionTwo(lastwa3,wa3,bottomDriveLeft);
        inversionTwo(lastwa4,wa4,bottomDriveRight);
*/
      
        //// implement the code i didnt fucking look at even though it was one scroll away god damn it
        //did and it worked
/*
        if(Math.abs(wa1) - Math.abs(topTurnRight.getSelectedSensorPosition()) > 90)
        {
            ws1 *= -1;
            wa1 = ((wa1 + 360) % 360) - 180;
        }
        if(Math.abs(wa2) > 90)
        {
            ws2 *= -1;
            wa2 = ((wa2 + 360) % 360) - 180;
        }
        if(Math.abs(wa3) > 90)
        {
            ws3 *= -1;
            wa3 = ((wa3 + 360) % 360) - 180;
        }
        if(Math.abs(wa4) > 90)
        {
            ws4 *= -1;
            wa4 = ((wa4 + 360) % 360) - 180;
        }
        */
/*
        inversionAwarness(topTurnLeft, wa2);
        inversionAwarness(topTurnRight, wa1);
        inversionAwarness(bottomTurnRight, wa4);
        inversionAwarness(bottomTurnLeft, wa3);
*/      

        /*
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(wa2, TURN.greerRatio))); //wa2
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(wa1, TURN.greerRatio))); //wa1
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(wa4, TURN.greerRatio))); //wa4
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(wa3, TURN.greerRatio))); //wa3
*/
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(wa2, TURN.greerRatio))); //wa2
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(wa1, TURN.greerRatio))); //wa1
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(wa4, TURN.greerRatio))); //wa4
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(wa3, TURN.greerRatio))); //wa3

        
        //// so many fucking things to test
            //yes, but not here

        SmartDashboard.putNumber("speedtopright", topDriveRight.getMotorOutputPercent());
        SmartDashboard.putNumber("speedtopleft", topDriveLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("speedbotleft", bottomDriveLeft.getMotorOutputPercent());
        SmartDashboard.putNumber("speedbotright", bottomDriveRight.getMotorOutputPercent());


        SmartDashboard.putNumber("ws1", ws1);
        SmartDashboard.putNumber("ws2", ws2);
        SmartDashboard.putNumber("ws3", ws3);
        SmartDashboard.putNumber("ws4", ws4);

        SmartDashboard.putNumber("ptopl", driveTopLeft.getP());
        SmartDashboard.putNumber("ptopr", driveTopRight.getP());
        SmartDashboard.putNumber("pbotl", driveBotLeft.getP());
        SmartDashboard.putNumber("pbotr", driveBotRight.getP());
    }

    //!             figure above shit later, make simple cave man code
  
  
    //!             caveman cannot figure out, figure out now


    //!             caveman now write basic bitch code cuz caveman a basic bitch


/*
    public void forwardStrafe(double leftX, double leftY)
    {
        double theta = Math.toDegrees(Math.atan2(leftY,leftX));
        if(theta < 0)
        {
            theta += 180; //// might be add 360 idk
                                    abandoned
        }
        double speed = Math.sqrt(Math.pow(leftX,2) + Math.pow(leftY,2));
        if(speed > 1)
        {
            speed = 1;
        }
        topDriveLeft.set(ControlMode.PercentOutput, speed);
        topDriveRight.set(ControlMode.PercentOutput, speed);
        bottomDriveRight.set(ControlMode.PercentOutput, speed);
        bottomDriveLeft.set(ControlMode.PercentOutput, speed);

        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(theta, TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(theta, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(theta, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(theta, TURN.greerRatio)));
    }
*/


    //!         do rotate later, figure out how og code vars work and react
    //!         also test handmade code as well









    public void turnEncoder(double setpoint)
    {
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeftEncoder(setpoint));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRightEncoder(setpoint));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRightEncoder(setpoint));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeftEncoder(setpoint));
    }   
    
    public void turnInternalEncoder(double setpoint)
    {



        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(setpoint, TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(setpoint, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(setpoint, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(setpoint, TURN.greerRatio)));
    }   

    public void turnRotateNavX()
    {
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(navXshit(), TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(navXshit(), TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(navXshit(), TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(navXshit(), TURN.greerRatio)));
    }

 
/*
    public void turnEncoderZero()
    {
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeftEncoder(offsetTopLeftCANCoder));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRightEncoder(offsetTopRightCANCoder));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeftEncoder(offsetBottomLeftCANCoder));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRightEncoder(offsetBottomRightCANCoder));
    }

*/




    public void ifZero()
    {
        if(Math.abs(MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio)) < 4
        && Math.abs(MkUtil.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), TURN.greerRatio)) < 4
        && Math.abs(MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio)) < 4
        && Math.abs(MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio)) < 4)
        {
            setTurnPos(0);
        }
    }


    

    public void zero()
    {
        //// use when we actually get the cancoders
        //dont use cancoders other than to get starting position
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(
            MkUtil.setDirection(topTurnLeft, 0, TURN.greerRatio), TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(
            MkUtil.setDirection(topTurnRight, 0, TURN.greerRatio), TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(
            MkUtil.setDirection(bottomTurnRight, 0, TURN.greerRatio), TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(
            MkUtil.setDirection(bottomTurnLeft, 0, TURN.greerRatio), TURN.greerRatio)));
    }

    public void zeroRobotNavx()
    {
        navX.reset();
         //// get tephlan and ask him how swerve work again
    }
                        //mouse, so funny
    public double semiAutonomouseTurn(double desiredAngle, double currentTime, double desiredTime, double spinInPlace)
    {
        double angle = currentTime * (desiredAngle/desiredTime);
        double FWD = Math.sin(angle);
        double STR = Math.cos(angle);
        double RCW = spinInPlace;
        double nav = navXshit();
        double temp = FWD*Math.cos(Math.toRadians(nav)) + STR*Math.sin(Math.toRadians(nav));
        STR = -FWD*Math.sin(Math.toRadians(nav)) + STR*Math.cos(Math.toRadians(nav));
        FWD = temp; 

        double A = STR - RCW*(Constants.L/Constants.R); //-
        double B = STR + RCW*(Constants.L/Constants.R); //+
        double C = FWD - RCW*(Constants.W/Constants.R); //-
        double D = FWD + RCW*(Constants.W/Constants.R); //+

        double ws1 = Math.sqrt(Math.pow(B,2)+Math.pow(C,2));   double wa1 = Math.atan2(B,C)*(180/Math.PI);
        double ws2 = Math.sqrt(Math.pow(B,2)+Math.pow(D,2));   double wa2 = Math.atan2(B,D)*(180/Math.PI);
        double ws3 = Math.sqrt(Math.pow(A,2)+Math.pow(D,2));   double wa3 = Math.atan2(A,D)*(180/Math.PI);
        double ws4 = Math.sqrt(Math.pow(A,2)+Math.pow(C,2));   double wa4 = Math.atan2(A,C)*(180/Math.PI);

       
        wa2 = MkUtil.setDirection(topTurnLeft, wa2, driveTopLeft);
        wa1 = MkUtil.setDirection(topTurnRight, wa1, driveTopRight);
        wa4 = MkUtil.setDirection(bottomTurnRight, wa4, driveBotRight);
        wa3 = MkUtil.setDirection(bottomTurnLeft, wa3, driveBotLeft);

        ws2 = MkUtil.isPositive(driveTopLeft.getP(), ws2);
        ws1 = MkUtil.isPositive(driveTopRight.getP(), ws1);
        ws4 = MkUtil.isPositive(driveBotRight.getP(), ws4);
        ws3 = MkUtil.isPositive(driveBotLeft.getP(), ws3);

        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(wa2, TURN.greerRatio))); //wa2
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(wa1, TURN.greerRatio))); //wa1
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(wa4, TURN.greerRatio))); //wa4
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(wa3, TURN.greerRatio))); //wa3

        if(angle == desiredAngle)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }











   

    public double currentDistance = 0;

    public double distanceA = 0;
    public double lengthB = 0;

    //let the math begin
    //in inches
    public double calculateCircleRadius(double distanceAA, double lengthBB)
    {
        return ((Math.pow(distanceAA, 2)/4) + Math.pow(lengthBB, 2)) * (1 / (2 * lengthBB));
    }
    public double calculateAngularVelocity(double distanceAA, double lengthBB)
    {
        double radius = calculateCircleRadius(distanceAA, lengthBB);
        return (DRIVE.maxInchesVelocity / radius);
    }
    public double calculateArcOfPath(double distanceAA, double lengthBB)
    {
        double radius = calculateCircleRadius(distanceAA, lengthBB);
        double theta = 2 * (Math.asin((distanceAA/(2 * radius))));
        return (theta / 360) * (2* (Constants.kPi * radius));
    }
    public double calculateAngleOfPath(double distanceAA, double lengthBB)
    {
        double radius = calculateCircleRadius(distanceAA, lengthBB);
        return 2 * (Math.asin((distanceAA/(2 * radius))));
    }


    public void autoTurnSet(double distanceAA, double lengthBB, double totalDistance)
    {
        currentDistance = 0;
    }
    public void autoTurnUpdate(double totalDistance, double thetaTurn)
    {
        currentDistance = 
            (MkUtil.nativeToDegrees(topDriveLeft.getSelectedSensorPosition(), TURN.greerRatio) +
            MkUtil.nativeToDegrees(topDriveRight.getSelectedSensorPosition(), TURN.greerRatio) +
            MkUtil.nativeToDegrees(bottomDriveLeft.getSelectedSensorPosition(), TURN.greerRatio) + 
            MkUtil.nativeToDegrees(bottomDriveRight.getSelectedSensorPosition(), TURN.greerRatio)) / 4;

        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(((currentDistance/totalDistance)*thetaTurn), TURN.greerRatio)));
    }
    public boolean autoTurnIsDone(double totalDistance)
    {
        return Math.abs(totalDistance - currentDistance) < 0.5 && Math.abs(avgVelInches) < 0.1;
    }















    private static class InstanceHolder
    {
        private static final Drive mInstance = new Drive();
    } 
}
