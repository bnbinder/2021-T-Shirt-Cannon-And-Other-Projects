// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;
import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LIGHTS;
/** Add your docs here. */
public class leds {
    private Timer timer = new Timer();
    private int offset;
    private Drive mDrive = Drive.getInstance();
    private AddressableLED LEDS = new AddressableLED(LIGHTS.PWMPORT);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LIGHTS.bufferNum);
    private int navXRot = 0;
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
          navXRot = (int)mDrive.navXshit();
          for(int i = navXRot - 30; i < 30 + navXRot; i++)
          {
            buffer.setRGB(i, 255, 255, 255);
          }
          LEDS.setData(buffer);
          
      }

    private static class InstanceHolder
    {
        private static final leds mInstance = new leds();
    } 
}
