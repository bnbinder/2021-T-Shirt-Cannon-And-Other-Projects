// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
    private final PIDController turning = new PIDController(TURN.turnKP, TURN.turnKI, TURN.turnKD);
    private final PIDController speedZero = new PIDController(DRIVE.driveKP, DRIVE.driveKI, DRIVE.driveKD);
    private final PIDController turningEncoder = new PIDController(TURN.turnEncoderKP, TURN.turnEncoderKI, TURN.turnEncoderKD);

    private double degrees = 0;

    private double lastwa1 = 0;
    private double lastwa2 = 0;
    private double lastwa3 = 0;
    private double lastwa4 = 0;
    
    private double offsetTopLeftCANCoder = topTurnRightEncoder.getAbsolutePosition();
    private double offsetTopRightCANCoder = topTurnRightEncoder.getAbsolutePosition();
    private double offsetBottomLeftCANCoder = bottomTurnLeftEncoder.getAbsolutePosition();
    private double offsetBottomRightCANCoder = bottomTurnRightEncoder.getAbsolutePosition();
    /*
    private double canIPutMy1 = 0;
    private double canIPutMy2 = 0;
    private double canIPutMy3 = 0;
    private double canIPutMy4 = 0;
    private double ballsInYoJaw = 0;
    */
    //failed attempt at unicorn drive

    private double distance;
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    SwerveModuleState frontLeft = new SwerveModuleState();
    SwerveModuleState frontRight = new SwerveModuleState();
    SwerveModuleState bottomLeft = new SwerveModuleState();
    SwerveModuleState bottomRight = new SwerveModuleState();

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, navXshit2(), new Pose2d(0, 0, new Rotation2d()));

ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
SwerveModuleState states[] = m_kinematics.toSwerveModuleStates(speeds);

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
                   
                   avgVelInches, avgDistInches;

                   
    
    private Drive()
    { //TODO set integrated sensor for all thingies 
        topDriveLeft.configFactoryDefault();
        bottomDriveLeft.configFactoryDefault();
        topDriveRight.configFactoryDefault();
        bottomDriveRight.configFactoryDefault();

        topTurnLeft.configFactoryDefault();
        bottomTurnLeft.configFactoryDefault();
        topTurnRight.configFactoryDefault();
        bottomTurnRight.configFactoryDefault();
    
        topDriveLeft.setNeutralMode(NeutralMode.Coast);
        topDriveRight.setNeutralMode(NeutralMode.Coast);
        bottomDriveLeft.setNeutralMode(NeutralMode.Coast);
        bottomDriveRight.setNeutralMode(NeutralMode.Coast);
        
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



        topDriveLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        topDriveLeft.configVelocityMeasurementWindow(32);

        topDriveRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        topDriveRight.configVelocityMeasurementWindow(32);

        bottomDriveLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        bottomDriveLeft.configVelocityMeasurementWindow(32);

        bottomDriveRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        bottomDriveRight.configVelocityMeasurementWindow(32);



        topTurnLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        topTurnLeft.configVelocityMeasurementWindow(32);

        topTurnRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        topTurnRight.configVelocityMeasurementWindow(32);

        bottomTurnLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        bottomTurnLeft.configVelocityMeasurementWindow(32);

        bottomTurnRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        bottomTurnRight.configVelocityMeasurementWindow(32);


        
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


        topDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        topDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        topTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        topTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        topDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        topDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        topTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        topTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        bottomDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        bottomDriveLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        bottomTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        bottomTurnLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        bottomDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        bottomDriveRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

        bottomTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        bottomTurnRight.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);


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

        topTurnLeftEncoder.configFactoryDefault();
        topTurnRightEncoder.configFactoryDefault();
        bottomTurnLeftEncoder.configFactoryDefault();
        bottomTurnRightEncoder.configFactoryDefault();

        topTurnLeftEncoder.configMagnetOffset(offsetTopLeftCANCoder);
        topTurnRightEncoder.configMagnetOffset(offsetTopRightCANCoder);
        bottomTurnLeftEncoder.configMagnetOffset(offsetBottomLeftCANCoder);
        bottomTurnRightEncoder.configMagnetOffset(offsetBottomRightCANCoder);
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
        //TODO for avg dist need to figure out how to do avg dist since four motors four wheels not tank drive
        //TODO same with avg vel
        /*
        SmartDashboard.putNumber("1top turn right", canIPutMy1);
        SmartDashboard.putNumber("2top turn left", canIPutMy2);
        SmartDashboard.putNumber("3bot turn right", canIPutMy3);
        SmartDashboard.putNumber("4bot turn left", canIPutMy4);

        SmartDashboard.putNumber("mr", ballsInYoJaw);
        */

        //degrees = MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio);
        SmartDashboard.putNumber("topledeg", MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio));
        SmartDashboard.putNumber("toprideg", MkUtil.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), TURN.greerRatio));
        SmartDashboard.putNumber("botledeg", MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio));
        SmartDashboard.putNumber("botrideg", MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio));
        //(frontLeft,frontRight,bottomLeft,bottomRight);
        frontLeft = new SwerveModuleState(leftTopVelMeters, Rotation2d.fromDegrees(MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio)));
        frontRight = new SwerveModuleState(rightTopVelMeters, Rotation2d.fromDegrees(MkUtil.nativeToDegrees(topTurnRight.getSelectedSensorPosition(), TURN.greerRatio)));
        bottomLeft = new SwerveModuleState(leftBottomVelMeters, Rotation2d.fromDegrees(MkUtil.nativeToDegrees(bottomTurnLeft.getSelectedSensorPosition(), TURN.greerRatio)));
        bottomRight = new SwerveModuleState(rightBottomVelMeters, Rotation2d.fromDegrees(MkUtil.nativeToDegrees(bottomTurnRight.getSelectedSensorPosition(), TURN.greerRatio)));
        m_odometry.update(navXshit2(), frontLeft,frontRight,bottomLeft,bottomRight);
        SmartDashboard.putNumber("x", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", m_odometry.getPoseMeters().getY());

        SmartDashboard.putNumber("navx", navX.getYaw());

        SmartDashboard.putBoolean("encoder t left", topTurnLeft.getInverted());
        SmartDashboard.putBoolean("encoder t right", topTurnRight.getInverted());
        SmartDashboard.putBoolean("encoder b left", bottomTurnLeft.getInverted());
        SmartDashboard.putBoolean("encoder b right", bottomTurnRight.getInverted());
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
        return turning.calculate(topTurnLeft.getSelectedSensorPosition(), setpoint);
    }
    public double turnCalculateTopRight(double setpoint)
    {
        return turning.calculate(topTurnRight.getSelectedSensorPosition(), setpoint);
    }
    public double turnCalculateBotLeft(double setpoint)
    {
        return turning.calculate(bottomTurnLeft.getSelectedSensorPosition(), setpoint);
    }
    public double turnCalculateBotRight(double setpoint)
    {
        return turning.calculate(bottomTurnRight.getSelectedSensorPosition(), setpoint);
    }





    public double turnCalculateTopLeftEncoder(double setpoint)
    {
        return turningEncoder.calculate(topTurnLeftEncoder.getAbsolutePosition(), setpoint);
    }
    public double turnCalculateTopRightEncoder(double setpoint)
    {
        return turningEncoder.calculate(topTurnRightEncoder.getAbsolutePosition(), setpoint);
    }
    public double turnCalculateBotLeftEncoder(double setpoint)
    {
        return turningEncoder.calculate(bottomTurnLeftEncoder.getAbsolutePosition(), setpoint);
    }
    public double turnCalculateBotRightEncoder(double setpoint)
    {
        return turningEncoder.calculate(bottomTurnRightEncoder.getAbsolutePosition(), setpoint);
    }





    public double turnDegCalculate(double setpoint)
    {
        setpoint = MkUtil.degreesToNative(setpoint, TURN.greerRatio);
        return turning.calculate(topTurnLeft.getSelectedSensorPosition(), setpoint);
    }

    //TODO may not need these


    public void zeroSensors() 
    {
        navX.zeroYaw();
        topDriveLeft.setSelectedSensorPosition(0);
        topDriveRight.setSelectedSensorPosition(0);
        bottomDriveLeft.setSelectedSensorPosition(0);
        bottomDriveRight.setSelectedSensorPosition(0);
    }    

    public void setMagicStraight(double setpoint)
    {
        distance = setpoint;

        topDriveLeft.configMotionCruiseVelocity(DRIVE.magicVel);
        topDriveRight.configMotionCruiseVelocity(DRIVE.magicVel);
        bottomDriveLeft.configMotionCruiseVelocity(DRIVE.magicVel);
        bottomDriveRight.configMotionCruiseVelocity(DRIVE.magicVel);

        topDriveLeft.configMotionAcceleration(DRIVE.magicAccel);
        topDriveRight.configMotionAcceleration(DRIVE.magicAccel);
        bottomDriveLeft.configMotionAcceleration(DRIVE.magicAccel);
        bottomDriveRight.configMotionAcceleration(DRIVE.magicAccel);

        zeroSensors();
    }

    public void updateMagicStraight()
    {
        topDriveLeft.set(ControlMode.MotionMagic, distance);
        topDriveRight.set(ControlMode.MotionMagic, distance);
        bottomDriveLeft.set(ControlMode.MotionMagic, distance);
        bottomDriveRight.set(ControlMode.MotionMagic, distance);
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

        //TODO add these ifs after testing if constraints fuck things up

        //sus about this, very sus
        //nvm its good doesnt break the code
        double max=ws1; if(ws2>max)max=ws2; if(ws3>max)max=ws3; if(ws4>max)max=ws4;
        if(max>1){ws1/=max; ws2/=max; ws3/=max; ws4/=max;}

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
        wa2 = MkUtil.setDirection(topTurnLeft, wa2, TURN.greerRatio);
        wa1 = MkUtil.setDirection(topTurnRight, wa1, TURN.greerRatio);
        wa4 = MkUtil.setDirection(bottomTurnRight, wa4, TURN.greerRatio);
        wa3 = MkUtil.setDirection(bottomTurnLeft, wa3, TURN.greerRatio);
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

        
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(wa2, TURN.greerRatio))); //wa2
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(wa1, TURN.greerRatio))); //wa1
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(wa4, TURN.greerRatio))); //wa4
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(wa3, TURN.greerRatio))); //wa3

        
        //TODO so many fucking things to test

        SmartDashboard.putNumber("topturnleft", turnCalculateTopLeft(MkUtil.degreesToNative(wa2, TURN.greerRatio)));
        SmartDashboard.putNumber("topturnright", turnCalculateTopRight(MkUtil.degreesToNative(wa1, TURN.greerRatio)));
        SmartDashboard.putNumber("bottomturnright", turnCalculateBotRight(MkUtil.degreesToNative(wa4, TURN.greerRatio)));
        SmartDashboard.putNumber("bottomturnleft", turnCalculateBotLeft(MkUtil.degreesToNative(wa3, TURN.greerRatio)));


        SmartDashboard.putNumber("wa1", wa1);
        SmartDashboard.putNumber("wa2", wa2);
        SmartDashboard.putNumber("wa3", wa3);
        SmartDashboard.putNumber("wa4", wa4);

        SmartDashboard.putNumber("lastwa1", lastwa1);
        SmartDashboard.putNumber("lastwa2", lastwa2);
        SmartDashboard.putNumber("lastwa3", lastwa3);
        SmartDashboard.putNumber("lastwa4", lastwa4);
    }

    //!             figure above shit later, make simple cave man code
  
  
    //!             caveman cannot figure out, figure out now


    //!             caveman now write basic bitch code cuz caveman a basic bitch



    public void forwardStrafe(double leftX, double leftY)
    {
        double theta = Math.toDegrees(Math.atan2(leftY,leftX));
        if(theta < 0)
        {
            theta += 180; //TODO might be add 360 idk
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



    //!         do rotate later, figure out how og code vars work and react
    //!         also test handmade code as well









    public void turnNine()
    {
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(90, TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(90, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(90, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(90, TURN.greerRatio)));
    }   
    
    public void turnEight()
    {
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(180, TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(180, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(180, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(180, TURN.greerRatio)));
    }   

    public void turnSeventy()
    {
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(270, TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(270, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(270, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(270, TURN.greerRatio)));
    } 


    public void turnZero()
    {
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(0, TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(0, TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(0, TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(0, TURN.greerRatio)));
    }
    public void turnRotateNavX()
    {
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeft(MkUtil.degreesToNative(navXshit(), TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRight(MkUtil.degreesToNative(navXshit(), TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRight(MkUtil.degreesToNative(navXshit(), TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeft(MkUtil.degreesToNative(navXshit(), TURN.greerRatio)));
    }

 

    public void turnEncoderZero()
    {
        topTurnLeft.set(ControlMode.PercentOutput, turnCalculateTopLeftEncoder(MkUtil.degreesToNative(topTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio)));
        topTurnRight.set(ControlMode.PercentOutput, turnCalculateTopRightEncoder(MkUtil.degreesToNative(topTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio)));
        bottomTurnLeft.set(ControlMode.PercentOutput, turnCalculateBotLeftEncoder(MkUtil.degreesToNative(bottomTurnLeftEncoder.getAbsolutePosition(), TURN.greerRatio)));
        bottomTurnRight.set(ControlMode.PercentOutput, turnCalculateBotRightEncoder(MkUtil.degreesToNative(bottomTurnRightEncoder.getAbsolutePosition(), TURN.greerRatio)));
    }






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
        //TODO use when we actually get the cancoders
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
         //TODO get tephlan and ask him how swerve work again
    }

    private static class InstanceHolder
    {
        private static final Drive mInstance = new Drive();
    } 
}
