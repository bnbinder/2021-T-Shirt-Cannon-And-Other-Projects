// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner6;
import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LIGHTS;
/** Add your docs here. */
public class leds {
    private AddressableLED LEDS = new AddressableLED(LIGHTS.PWMPORT);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(LIGHTS.bufferNum);
    private leds()
    {
        LEDS.setLength(buffer.getLength());
        LEDS.setData(buffer);
        LEDS.start();
    }

    public static leds getInstance()
    {
        return InstanceHolder.mInstance;
    }
    
    public void french() {
        // For every pixel
        for (var i = 0; i < buffer.getLength(); i++) 
        {
            if(i < (buffer.getLength() / 3))
            {
                buffer.setRGB(i, 0, 0, 255);
            }
            else if(i >= (buffer.getLength() / 3) && i < ((2 * buffer.getLength()) / 3))
            {
                buffer.setRGB(i, 255, 255, 255);
            }
            else
            {
                buffer.setRGB(i, 255, 0, 0);
            }
            LEDS.setData(buffer);
        }
        // Increase by to make the rainbow "move"

      }

    private static class InstanceHolder
    {
        private static final leds mInstance = new leds();
    } 
}
