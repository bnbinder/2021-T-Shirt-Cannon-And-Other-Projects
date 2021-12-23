// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;
import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LIGHTS;
/** Add your docs here. */
public class leds {
    private Timer timer = new Timer();
    private int offset;
    private Drive mDrive = Drive.getInstance();
    private AddressableLED LEDS = new AddressableLED(LIGHTS.PWMPORT);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LIGHTS.bufferNum);
    private double navXRot = 0;

    private boolean yesManOne = false;
    private boolean yesManTwo = false;


    private leds()
    {
        LEDS.setLength(LIGHTS.bufferNum);
        LEDS.setData(buffer);
        LEDS.start();
    }

    public static leds getInstance()
    {
        return InstanceHolder.mInstance;
    }
    
    public void french() {
        // For every pixel
        
        for (var i = 0; i < LIGHTS.bufferNum; i++) 
        {   
                timer.start();
                if(i < (LIGHTS.bufferNum / 3))
                {
                    buffer.setRGB((i+offset)%LIGHTS.bufferNum, 0, 0, 255);
                }
                else if(i >= (LIGHTS.bufferNum / 3) && i < ((2 * LIGHTS.bufferNum) / 3))
                {
                    buffer.setRGB((i+offset)%LIGHTS.bufferNum, 255, 255, 255);
                }
                else
                {
                    buffer.setRGB((i+offset)% LIGHTS.bufferNum, 255, 0, 0);
                }
                if(timer.get() > 0.08)
                {
                    offset = (offset + 1) % LIGHTS.bufferNum;
                    timer.reset();
                    LEDS.setData(buffer);
                }
            
        }
        // Increase by to make the rainbow "move"

      }

      public void lilNavX()
      {
      
      
          navXRot = (((mDrive.navXshit() + 360) % 360) * LIGHTS.bufferNum/360);
          SmartDashboard.putNumber("navxrrr", navXRot);
          for(double i = 0; i < LIGHTS.bufferNum; i++)
          {
              buffer.setRGB(((int)i + 100) % 100, 0, 0, 0);
              if(!(i > (((navXRot + LIGHTS.bufferNum) % LIGHTS.bufferNum) + 5) || i < (((navXRot + LIGHTS.bufferNum) % LIGHTS.bufferNum) - 5)))
              {
                buffer.setRGB(((int)i + 100) % 100, 255, 255, 255);
              }
              if(navXRot < 5 || navXRot > 95)
              {
                for(double j = navXRot - 5; j < navXRot + 5; j++)
                {
                    buffer.setRGB(((int)j + 100) % 100, 255, 255, 255);
                }
              } 
                
          }
          LEDS.setData(buffer);
          
      }

      public void lilNavXTWO()
      {
        navXRot = (((mDrive.navXshit() + 360) % 360) * LIGHTS.bufferNum/360);
        SmartDashboard.putNumber("navxrrr", navXRot);
        for(int i = 0; i < LIGHTS.bufferNum; i++)
        {
            if(((i + 10 / 2 - (int)navXRot) + LIGHTS.bufferNum) % LIGHTS.bufferNum < 10)
            {
                buffer.setRGB(i, 255/2, 255/2, 255/2);
            }
            else
            {
                buffer.setRGB(i, 0, 0, 0);
            }
        }
        LEDS.setData(buffer);
      }

      public void voltage(double volts)
      {
          for(int i = 0; i < LIGHTS.bufferNum; i++)
          {
              buffer.setRGB(i, (int)(((((volts - 11)/13) * 255))/2), 0, 0);
          }
          LEDS.setData(buffer);
      }

      public void rickRoss(int rickroller)
      {
          for(int i = 0; i < LIGHTS.bufferNum; i++)
          {
              buffer.setRGB(i, 0, rickroller, 0);
          }
      }

    private static class InstanceHolder
    {
        private static final leds mInstance = new leds();
    } 
}
