// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Scanner;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.UnsupportedAudioFileException;

public class WaveFile {
    public final int NOT_SPECIFIED = AudioSystem.NOT_SPECIFIED; // -1
    public final int INT_SIZE = 4;

    private int sampleSize = NOT_SPECIFIED;
    private long framesCount = NOT_SPECIFIED;
    private int sampleRate = NOT_SPECIFIED;
    private int channelsNum;
    private byte[] data;      // wav bytes
    private AudioInputStream ais;
    private AudioFormat af;

    private Clip clip;
    private boolean canPlay;

    public WaveFile(File file) throws UnsupportedAudioFileException, IOException {
        if (!file.exists()) {
            throw new FileNotFoundException(file.getAbsolutePath());
        }

        ais = AudioSystem.getAudioInputStream(file);

        af = ais.getFormat();

        framesCount = ais.getFrameLength();

        sampleRate = (int) af.getSampleRate();

        sampleSize = af.getSampleSizeInBits() / 8;

        channelsNum = af.getChannels();

        long dataLength = framesCount * af.getSampleSizeInBits() * af.getChannels() / 8;

        data = new byte[(int) dataLength];
        ais.read(data);

        AudioInputStream aisForPlay = AudioSystem.getAudioInputStream(file);
        try {
            clip = AudioSystem.getClip();
            clip.open(aisForPlay);
            clip.setFramePosition(0);
            canPlay = true;
        } catch (LineUnavailableException e) {
            canPlay = false;
            System.out.println("I can play only 8bit and 16bit music.");
        }
    }

    public boolean isCanPlay() {
        return canPlay;
    }

    public void play() {
        clip.start();
    }

    public void stop() {
        clip.stop();
    }

    public AudioFormat getAudioFormat() {
        return af;
    }

    public int getSampleSize() {
        return sampleSize;
    }

    public double getDurationTime() {
        return getFramesCount() / getAudioFormat().getFrameRate();
    }

    public long getFramesCount() {
        return framesCount;
    }


    /**
     * Returns sample (amplitude value). Note that in case of stereo samples
     * go one after another. I.e. 0 - first sample of left channel, 1 - first
     * sample of the right channel, 2 - second sample of the left channel, 3 -
     * second sample of the rigth channel, etc.
     */
    public int getSampleInt(int sampleNumber) {

        if (sampleNumber < 0 || sampleNumber >= data.length / sampleSize) {
            throw new IllegalArgumentException(
                    "sample number can't be < 0 or >= data.length/"
                            + sampleSize);
        }

        byte[] sampleBytes = new byte[4]; //4byte = int

        for (int i = 0; i < sampleSize; i++) {
            sampleBytes[i] = data[sampleNumber * sampleSize * channelsNum + i];
        }

        int sample = ByteBuffer.wrap(sampleBytes)
                .order(ByteOrder.LITTLE_ENDIAN).getInt();
        return sample;
    }

    public int getSampleRate() {
        return sampleRate;
    }

    public Clip getClip() {
        return clip;
    }

    public static void main(String[] args)
	{
		try
		{
            
			File file = new File("c:/Users/bossMaster/Desktop/thhhh.wav");
			WaveFile wav = new WaveFile(file);
			Scanner sc = new Scanner(System.in);
            wav.play();


// Begin audio capture.

// Here, stopped is a global boolean set by another thread.
/*
while (!stopped) {
   // Read the next chunk of data from the TargetDataLine.
   numBytesRead =  line.read(data, 0, data.length);
   // Save this chunk of data.
   out.write(data, 0, numBytesRead);
}     
*/
			while (true)
			{
				
				System.out.println("1. pause");
				System.out.println("2. resume");
				System.out.println("3. restart");
				System.out.println("4. stop");
				System.out.println("5. Jump to specific time");
				int c = sc.nextInt();
				wav.gotoChoice(c);
				if (c == 4)
                wav.stop();
                //wav.getClip().flush();
                //wav.getClip().drain();
				break;
			}
			sc.close();
		}
		
		catch (Exception ex)
		{
			System.out.println("Error with playing sound.");
			ex.printStackTrace();
		
		}
	}
	
	// Work as the user enters his choice
	
	private void gotoChoice(int c)
			throws IOException, LineUnavailableException, UnsupportedAudioFileException
	{
		switch (c)
	    {
			case 6:
				
				//System.out.println(getBytesPositionFromMilliseconds(clip.getMicrosecondPosition(), 2, 2, (int)clip.getFormat().getSampleRate()));
				//System.out.println(clip.getFrameLength());
				System.out.println(getSampleInt(140));
				break;
                
            
		}
	
	
    }

   public int stuff()
    {
        return getSampleInt(140);
    }

}