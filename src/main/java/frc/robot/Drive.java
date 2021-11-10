// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DRIVE;

/** Add your docs here. */
public class Drive {
    TalonFX topDriveRight = new TalonFX(DRIVE.topDriveRightCANID);
    TalonFX topTurnRight = new TalonFX(DRIVE.topTurnRightCANID);
    
    TalonFX bottomDriveRight = new TalonFX(DRIVE.bottomDriveRightCANID);
    TalonFX bottomTurnRight = new TalonFX(DRIVE.bottomTurnRightCANID);
   
    TalonFX topDriveLeft = new TalonFX(DRIVE.topDriveLeftCANID);
    TalonFX topTurnLeft = new TalonFX(DRIVE.topTurnLeftCANID);
    
    TalonFX bottomDriveLeft = new TalonFX(DRIVE.bottomDriveLeftCANID);
    TalonFX bottomTurnLeft = new TalonFX(DRIVE.bottomTurnLeftCANID);

    private double distance;
    private double leftTopPosNative, leftBottomPosNative, 
                   rightTopPosNative, rightBottomPosNative,
                   
                   leftTopPosInch, leftBottomPosInch,
                   rightTopPosInch, rightBottomPosInch,
                   
                   leftTopVelInch, leftBottomVelInch,
                   rightTopVelInch, rightBottomVelInch,

                   leftTopVelNative, leftBottomVelNative,
                   rightTopVelNative, rightBottomVelNative;
    
    private Drive()
    {
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
    }

    public static Drive getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void updateDrive()
    {
        SmartDashboard.putNumber("top turn right", topTurnRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("top turn left", topTurnLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("bot turn right", bottomTurnRight.getSelectedSensorPosition());
        SmartDashboard.putNumber("bot turn left", bottomTurnLeft.getSelectedSensorPosition());


        topDriveLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        topTurnLeft.configVelocityMeasurementWindow(32);

        topDriveRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        topTurnRight.configVelocityMeasurementWindow(32);

        bottomDriveLeft.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        bottomTurnLeft.configVelocityMeasurementWindow(32);

        bottomDriveRight.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
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


        topTurnLeft.configPeakOutputForward(.20);
        topTurnLeft.configPeakOutputReverse(.20);

        topTurnRight.configPeakOutputForward(.20);
        topTurnRight.configPeakOutputReverse(.20);

        bottomTurnLeft.configPeakOutputForward(.20);
        bottomTurnLeft.configPeakOutputReverse(.20);

        bottomTurnRight.configPeakOutputForward(.20);
        bottomTurnRight.configPeakOutputReverse(.20);

        //what this do?
        //topTurnRight.configAllowableClosedloopError(0, 1);

        //makes sure motors dont go above 1 (100%)?
        topTurnLeft.configClosedLoopPeakOutput(0, .20);
        topTurnRight.configClosedLoopPeakOutput(0, .20);
        bottomTurnLeft.configClosedLoopPeakOutput(0, .20);
        bottomTurnRight.configClosedLoopPeakOutput(0, .20);

        //during open loop, 0.5 seconds before motor go from 0 to selected output
        topDriveLeft.configOpenloopRamp(DRIVE.driveOpenRampRate);
        topDriveRight.configOpenloopRamp(DRIVE.driveOpenRampRate);
        bottomDriveLeft.configOpenloopRamp(DRIVE.driveOpenRampRate);
        bottomDriveRight.configOpenloopRamp(DRIVE.driveOpenRampRate);

        //during close loop, 0.5 seconds before motor go from 0 to selected output
        topDriveLeft.configClosedloopRamp(DRIVE.driveCloseRampRate);
        topDriveRight.configClosedloopRamp(DRIVE.driveCloseRampRate);
        bottomDriveLeft.configClosedloopRamp(DRIVE.driveCloseRampRate);
        bottomDriveRight.configClosedloopRamp(DRIVE.driveCloseRampRate);
    }

    public void setTurnPos(double position)
    {
        topTurnLeft.setSelectedSensorPosition(position);
        topTurnRight.setSelectedSensorPosition(position);
        bottomTurnLeft.setSelectedSensorPosition(position);
        bottomTurnRight.setSelectedSensorPosition(position);
    }

    public void setTwoTurnPos(double position)
    {
        topTurnLeft.set(ControlMode.Position, position);
        topTurnRight.set(ControlMode.Position, position);
        bottomTurnLeft.set(ControlMode.Position, position);
        bottomTurnRight.set(ControlMode.Position, position);
    }

    public void setTurnPercent(double topleft, double topright, double botleft, double botright)
    {
        topTurnLeft.set(ControlMode.Position, topleft);
        topTurnRight.set(ControlMode.Position, topright);
        bottomTurnLeft.set(ControlMode.Position, botleft);
        bottomTurnRight.set(ControlMode.Position, botright);
    }

    private static class InstanceHolder
    {
        private static final Drive mInstance = new Drive();
    } 
}
