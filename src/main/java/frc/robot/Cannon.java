// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants.CANNON;
/** Add your docs here. */
public class Cannon {
    



/*
    
    private Compressor compressor = new Compressor(CANNON.compressorID);
    private Solenoid solenoidOne = new Solenoid(CANNON.solenoidOneID);
    private Solenoid solenoidTwo = new Solenoid(CANNON.solenoidTwoID);
    private TalonSRX genevaConvention = new TalonSRX(CANNON.genevaConventionCANID);
    private TalonSRX trunnion = new TalonSRX(CANNON.trunnionCANID);
    //i did research on cannons. gotta know ur shit bro

    private Timer genevaTimer = new Timer();

    private PIDController PIDtrunnion = new PIDController(CANNON.trunnionKP, CANNON.trunnionKI, CANNON.trunnionKD);

    public int slot;

    //private TimerTask
    //!found this goo gaw when sifting through the different timers, neat. may look into using this sometime

    private Cannon() //TODO this may fuck things up, in swerd code it was public in all solenoid related things
    {
        solenoidOne.set(false);
        solenoidTwo.set(false);
        compressorStart();

        genevaConvention.configFactoryDefault();
        genevaConvention.configVoltageCompSaturation(CANNON.voltComp);
        genevaConvention.enableVoltageCompensation(true);
        genevaConvention.setNeutralMode(NeutralMode.Brake);

        trunnion.configFactoryDefault();
        trunnion.configVoltageCompSaturation(CANNON.voltComp);
        trunnion.enableVoltageCompensation(true);
        trunnion.setNeutralMode(NeutralMode.Brake);

        genevaTimer.reset();
        
    }

    public static Cannon getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public void compressorStart()
    {
        compressor.start();
    }

    public void compressorStop()
    {
        compressor.stop();
    }

    public void genevaSpin()
    {

    }

    public void russianRoulette()
    {
        slot = (int)((Math.random() * 6) + 1);
    }

    /*
    public double trunnionCalculate(double setpoint, mode modes)
    {
        switch(modes)
        {
            case angle:
            return PIDtrunnion.calculate(measurement, setpoint);

            case internal:
            return PIDtrunnion.calculate(measurement, setpoint)
        }
    }
    */
    /*
    public void trunnionFeedOptionOne(double setpointDegrees)
    {
                                    //degrees or native, probably native
        double kMeasuredPosHorizontal = CANNON.horizontalPosition; //Position measured when arm is horizontal

        double kTicksPerDegree = 4096 * CANNON.trunnionGreerRatio / 360; //Sensor is 1:1 with arm rotation

                                    //also native
        double currentPos = _motorcontroller.getSelectedSensorPosition();
        
        double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
        
        double radians = Math.toRadians(degrees);
        
        double cosineScalar = Math.cos(radians);
        
             //is this min power for arm to move?
        double maxGravityFF = 0.07;

                                         //is set point degrees or native, probably degrees
        trunnion.set(ControlMode.MotionMagic, setpointDegrees, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);
    }
    */
    /*
    public void trunnionFeedOptionTwo(double setpointDegrees)
    {                                                                                              //is theta current angle or setpoint angle?
        double FF = (Arm Weight) * (Distance to arm center of mass) / (Motor stall torque) * (Number of motors) * (Gear Ratio) * cos(theta);

                                                                    //figure out torque with stephan
        double FF = (CANNON.weight) * (CANNON.cannonDistance) / (Motor stall torque) * (1) * (CANNON.trunnionGreerRatio) * Math.cos(theta);
    }
    public enum mode
    {
        angle, internal
    }

    private static class InstanceHolder
    {
        private static final Cannon mInstance = new Cannon();
    } 
    */


    
}
