// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
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

    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(TURN.maxVel, TURN.maxAccel);
    private final ProfiledPIDController turning = new ProfiledPIDController(TURN.turnKP, TURN.turnKI, TURN.turnKD, constraints);

    private double degrees = 0;

    private double distance;
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


        topDriveLeft.enableVoltageCompensation(true);
        topDriveLeft.configVoltageCompSaturation(10);

        topTurnLeft.enableVoltageCompensation(true);
        topTurnLeft.configVoltageCompSaturation(10);

        topDriveRight.enableVoltageCompensation(true);
        topDriveRight.configVoltageCompSaturation(10);

        topTurnRight.enableVoltageCompensation(true);
        topTurnRight.configVoltageCompSaturation(10);

        bottomDriveLeft.enableVoltageCompensation(true);
        bottomDriveLeft.configVoltageCompSaturation(10);

        bottomTurnLeft.enableVoltageCompensation(true);
        bottomTurnLeft.configVoltageCompSaturation(10);

        bottomDriveRight.enableVoltageCompensation(true);
        bottomDriveRight.configVoltageCompSaturation(10);

        bottomTurnRight.enableVoltageCompensation(true);
        bottomTurnRight.configVoltageCompSaturation(10);

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
        SmartDashboard.putNumber("top turn right", topTurnRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("top turn left", topTurnLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("bot turn right", bottomTurnRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("bot turn left", bottomTurnLeft.getSelectedSensorPosition());

        degrees = MkUtil.nativeToDegrees(topTurnLeft.getSelectedSensorPosition(), TURN.greerRatio);
        SmartDashboard.putNumber("deg", degrees);
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

    public double turnCalculate(double setpoint)
    {
        return turning.calculate(topTurnLeft.getSelectedSensorPosition(), setpoint);
    }

    public void zeroSensors() {
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

    private static class InstanceHolder
    {
        private static final Drive mInstance = new Drive();
    } 
}
