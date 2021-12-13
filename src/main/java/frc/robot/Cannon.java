// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.TimerTask;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    //private TimerTask
    //!found this goo gaw when sifting through the different timers, neat. may look into using this sometime

    private Cannon() //TODO this may fuck things up, in swerd code it was public in all solenoid related things
    {
        solenoidOne.set(false);
        solenoidTwo.set(false);

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

    private static class InstanceHolder
    {
        private static final Cannon mInstance = new Cannon();
    } 
    


    */
}
