// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.archived_project;
import javax.swing.SwingUtilities;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JComponent;

import java.awt.BorderLayout;
import java.awt.Graphics;
import java.io.File;
import java.io.IOException;
import java.awt.Color;
import java.awt.Dimension;
import javax.swing.border.EmptyBorder;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioPermission;
import javax.sound.sampled.TargetDataLine;
import javax.sound.sampled.UnsupportedAudioFileException;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.SourceDataLine;

public class LevelMeter extends JComponent {
    private int meterWidth = 10;

    private float amp = 0f;
    private float peak = 0f;

    public void setAmplitude(float amp) {
        this.amp = Math.abs(amp);
        repaint();
    }

    public void setPeak(float peak) {
        this.peak = Math.abs(peak);
        repaint();
    }

    public void setMeterWidth(int meterWidth) {
        this.meterWidth = meterWidth;
    }

    @Override
    protected void paintComponent(Graphics g) {
        int w = Math.min(meterWidth, getWidth());
        int h = getHeight();
        int x = getWidth() / 2 - w / 2;
        int y = 0;

        g.setColor(Color.LIGHT_GRAY);
        g.fillRect(x, y, w, h);

        g.setColor(Color.BLACK);
        g.drawRect(x, y, w - 1, h - 1);

        int a = Math.round(amp * (h - 2));
        g.setColor(Color.GREEN);
        g.fillRect(x + 1, y + h - 1 - a, w - 2, a);

        int p = Math.round(peak * (h - 2));
        g.setColor(Color.RED);
        g.drawLine(x + 1, y + h - 1 - p, x + w - 1, y + h - 1 - p);
    }

    @Override
    public Dimension getMinimumSize() {
        Dimension min = super.getMinimumSize();
        if(min.width < meterWidth)
            min.width = meterWidth;
        if(min.height < meterWidth)
            min.height = meterWidth;
        return min;
    }

    @Override
    public Dimension getPreferredSize() {
        Dimension pref = super.getPreferredSize();
        pref.width = meterWidth;
        return pref;
    }

    @Override
    public void setPreferredSize(Dimension pref) {
        super.setPreferredSize(pref);
        setMeterWidth(pref.width);
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                JFrame frame = new JFrame("Meter");
                frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

                JPanel content = new JPanel(new BorderLayout());
                content.setBorder(new EmptyBorder(25, 50, 25, 50));

                LevelMeter meter = new LevelMeter();
                meter.setPreferredSize(new Dimension(9, 100));
                content.add(meter, BorderLayout.CENTER);

                frame.setContentPane(content);
                frame.pack();
                frame.setLocationRelativeTo(null);
                frame.setVisible(true);

                new Thread(new Recorder(meter)).start();
            }
        });
    }

    static class Recorder implements Runnable {
        final LevelMeter meter;

        Recorder(final LevelMeter meter) {
            this.meter = meter;
        }

        @Override
        public void run() {
            Clip clip;
            AudioInputStream audioInputStream;
            AudioInputStream audioBitch;
            try {
                audioInputStream = AudioSystem.getAudioInputStream(new File("C:/Users/bossMaster/Desktop/h.wav").getAbsoluteFile());
                audioBitch = AudioSystem.getAudioInputStream(new File("C:/Users/bossMaster/Desktop/h.wav").getAbsoluteFile());
                
            } catch (UnsupportedAudioFileException | IOException e1) {
                // TODO Auto-generated catch block
                e1.printStackTrace();
                return;
            }
            try {
                clip = AudioSystem.getClip();
            } catch (LineUnavailableException e1) {
                // TODO Auto-generated catch block
                e1.printStackTrace();
                return;
            }
            AudioFormat fmt = clip.getFormat();
            
            SourceDataLine line;
            try {
                line = AudioSystem.getSourceDataLine(fmt);
                line.open(fmt);
            } catch(LineUnavailableException e) {
                System.err.println(e);
                return;
            }
            try {
                clip.open(audioInputStream);
            } catch (LineUnavailableException e1) {
                // TODO Auto-generated catch block
                e1.printStackTrace();
                return;
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                return;
            }
            clip.start();
            final int bufferByteSize = (int)audioBitch.getFrameLength()/260000;
        
            byte[] buf = new byte[bufferByteSize];
            float[] samples = new float[bufferByteSize / 2];

            float lastPeak = 0f;

            int position = (int) millis2Bytes(clip.getMicrosecondPosition(), clip.getFormat());
            System.out.println((int)audioBitch.getFrameLength());
            line.start();

            try { 
                
                for(int b; (b = audioBitch.read(buf,0,buf.length)) > -1;) {
                       System.out.println(clip.getFramePosition() * clip.getFormat().getFrameSize());
                       //System.out.println((int)(audioBitch.getFrameLength() * 4));
                       
                    // convert bytes to samples here
                    for(int i = 0, s = 0; i < b;) {
                        int sample = 0;

                        sample |= buf[i++] & 0xFF; // (reverse these two lines
                        sample |= buf[i++] << 8;   //  if the format is big endian)
                        //System.out.println(sample);
                        // normalize to range of +/-1.0f
                        samples[s++] = sample / 32768f;
                    }

                    float rms = 0f;
                    float peak = 0f;
                    
                    for(float sample : samples) {

                        float abs = Math.abs(sample);
                        if(abs > peak) {
                            peak = abs;
                        }

                        rms += sample * sample;
                    }

                    rms = (float)Math.sqrt(rms / samples.length);

                    if(lastPeak > peak) {
                        peak = lastPeak * 0.875f;
                    }
                    //System.out.println(rms);
                    lastPeak = peak;

                    setMeterOnEDT(rms, peak);
                }
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
                return;
            }
        }

        void setMeterOnEDT(final float rms, final float peak) {
            SwingUtilities.invokeLater(new Runnable() {
                @Override
                public void run() {
                    meter.setAmplitude(rms);
                    meter.setPeak(peak);
                }
            });
        }
    }

    public float getAmp()
    {
        return amp;
    }

    public float getPeak()
    {
        return peak;
    }

    public static long millis2Bytes(long ms, AudioFormat format) {
        return millis2Bytes(ms, format.getFrameRate(), format.getFrameSize());
    }

    public static long millis2Bytes(long ms, double frameRate, int frameSize) {
        return (long) (ms * frameRate / 1000 * frameSize);
    }

    public static long millis2Bytes(double ms, AudioFormat format) {
        return millis2Bytes(ms, format.getFrameRate(), format.getFrameSize());
    }

    public static long millis2Bytes(double ms, double frameRate, int frameSize) {
        return ((long) (ms * frameRate / 1000.0)) * frameSize;
    }

}