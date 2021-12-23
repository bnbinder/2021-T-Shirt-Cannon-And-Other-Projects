// Java program to play an Audio
// file using Clip Object

package frc.robot;
import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.FloatControl;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.UnsupportedAudioFileException;

import edu.wpi.first.wpilibj.Timer;

public class SimpleAudioPlayer
{


	// to store current position
	Long currentFrame;
	Clip clip;
	
	// current status of clip
	String status;
	
	AudioInputStream audioInputStream;
	static String filePath = "C:/Users/bossMaster/Desktop/VXOP4165_1.wav";

	// constructor to initialize streams and clip
	private SimpleAudioPlayer()
		throws UnsupportedAudioFileException,
		IOException, LineUnavailableException
	{
		// create AudioInputStream object
		audioInputStream =
				AudioSystem.getAudioInputStream(new File(filePath).getAbsoluteFile());
		
		// create clip reference
		clip = AudioSystem.getClip();
		
		// open audioInputStream to the clip
		clip.open(audioInputStream);
		
		clip.loop(Clip.LOOP_CONTINUOUSLY);
	}

	public static SimpleAudioPlayer getInstance()
    {
        return InstanceHolder.mInstance;
    }
	
	// Method to play the audio

	public void play()
	{
		//start the clip
		clip.start();
		
		status = "play";
	}
	
	// Method to pause the audio
	public void pause()
	{
		if (status.equals("paused"))
		{
			System.out.println("audio is already paused");
			return;
		}
		this.currentFrame =
		this.clip.getMicrosecondPosition();
		clip.stop();
		status = "paused";
	}
	
	// Method to resume the audio
	public void resumeAudio() throws UnsupportedAudioFileException,
								IOException, LineUnavailableException
	{
		if (status.equals("play"))
		{
			System.out.println("Audio is already "+
			"being played");
			return;
		}
		clip.close();
		resetAudioStream();
		clip.setMicrosecondPosition(currentFrame);
		this.play();
	}
	
	// Method to restart the audio
	public void restart() throws IOException, LineUnavailableException,
											UnsupportedAudioFileException
	{
		clip.stop();
		clip.close();
		resetAudioStream();
		currentFrame = 0L;
		clip.setMicrosecondPosition(0);
		this.play();
	}
	
	// Method to stop the audio
	public void stop() throws UnsupportedAudioFileException,
	IOException, LineUnavailableException
	{
		currentFrame = 0L;
		clip.stop();
		clip.close();
	}
	
	// Method to jump over a specific part
	public void jump(long c) throws UnsupportedAudioFileException, IOException,
														LineUnavailableException
	{
		if (c > 0 && c < clip.getMicrosecondLength())
		{
			clip.stop();
			clip.close();
			resetAudioStream();
			currentFrame = c;
			clip.setMicrosecondPosition(c);
			this.play();
		}
	}
	
	// Method to reset audio stream
	public void resetAudioStream() throws UnsupportedAudioFileException, IOException,
											LineUnavailableException
	{
		audioInputStream = AudioSystem.getAudioInputStream(
		new File(filePath).getAbsoluteFile());
		clip.open(audioInputStream);
		clip.loop(Clip.LOOP_CONTINUOUSLY);
	}

	public float getVolume() {
		FloatControl gainControl = (FloatControl) clip.getControl(FloatControl.Type.MASTER_GAIN);        
		return (float) Math.pow(10f, gainControl.getValue() / 20f);
	}

	public float getDeci() {
		FloatControl gainControl = (FloatControl) clip.getControl(FloatControl.Type.MASTER_GAIN);        
		return gainControl.getValue();
	}
	
	public void setVolume(float volume) {
		if (volume < 0f || volume > 1f)
			throw new IllegalArgumentException("Volume not valid: " + volume);
		FloatControl gainControl = (FloatControl) clip.getControl(FloatControl.Type.MASTER_GAIN);        
		gainControl.setValue(20f * (float) Math.log10(volume));
	}

	public void setPath(String path)
	{
		filePath = path;
	}

	private static class InstanceHolder
    {
        private static final SimpleAudioPlayer mInstance;
		static {
			try {
				mInstance = new SimpleAudioPlayer();
			} catch (Exception e) {
				throw new ExceptionInInitializerError(e);
			}
		}
    } 
}
