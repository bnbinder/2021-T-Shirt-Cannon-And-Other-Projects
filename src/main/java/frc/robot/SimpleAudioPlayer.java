// Java program to play an Audio
// file using Clip Object
package frc.robot;
import java.io.BufferedOutputStream;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Scanner;

import javax.naming.ldap.SortControl;
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.Control;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.FloatControl;
import javax.sound.sampled.Line;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.Mixer;
import javax.sound.sampled.SourceDataLine;
import javax.sound.sampled.TargetDataLine;
import javax.sound.sampled.UnsupportedAudioFileException;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimpleAudioPlayer
{

	// to store current position
	 Long currentFrame;
	 Clip clip;

	leds mLeds = leds.getInstance();
    FloatControl control;
	
	// current status of clip
	static String status;

     File file;

	static int i = 0;

    static boolean stopped = true;

	 InputStream input;
    
    public  TargetDataLine line;
    DataLine.Info info; // format is an AudioFormat object
	
	 AudioInputStream audioInputStream;
	 AudioInputStream audioBitch;
	 static String filePath;

	private static final float MAX_REPORTABLE_DB = 90.3087f;
    private static final float MAX_REPORTABLE_AMP = 32767f;

     ByteArrayOutputStream output;
     int numBytesRead;
     byte[] data;

	 SourceDataLine source;

	//static OutputStream output;

	// constructor to initialize streams and clip
	public SimpleAudioPlayer()
		
	{
try
{
        file = new File(filePath).getAbsoluteFile();
		// create AudioInputStream object
		audioInputStream = AudioSystem.getAudioInputStream(file);
		audioBitch = AudioSystem.getAudioInputStream(file);
		input = AudioSystem.getAudioInputStream(file);
		// create clip reference
		clip = AudioSystem.getClip();
		line = AudioSystem.getTargetDataLine(audioBitch.getFormat());
		// open audioInputStream to the clip
		data = audioInputStream.readAllBytes();
		clip.open(audioBitch);
		
		clip.loop(Clip.LOOP_CONTINUOUSLY);


        
		output = new ByteArrayOutputStream();
        

		
		line.open(audioBitch.getFormat());
		
		source = AudioSystem.getSourceDataLine(audioBitch.getFormat());
		source.open();
		source.start();
		//Control control = line.getControl(FloatControl.Type.VOLUME);
		//System.out.println(source.isControlSupported(FloatControl.Type.VOLUME));

		info = (DataLine.Info) line.getLineInfo();
		// data = new byte[1];

		 //audioInputStream.transferTo(output);
		
		//line = 
    //out  = new ByteArrayOutputStream();
        //data = new byte[line.getBufferSize() / 5];
                
        //clip.open(audioInputStream.getFormat(), audioInputStream.readAllBytes(), 1, clip.getBufferSize());
       // control = (FloatControl) clip.getControl(FloatControl.Type.);

       // Assume that the TargetDataLine, line, has already
// been obtained and opened.
}
catch(Exception e)
{
	throw new StackOverflowError("bitch");
}
	}

	public static void main(String[] args)
	{
		try
		{
			filePath = "c:/Users/bossMaster/Desktop/thhhh.wav";
			SimpleAudioPlayer audioPlayer =
							new SimpleAudioPlayer();
			
			audioPlayer.play();
			Scanner sc = new Scanner(System.in);


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
				audioPlayer.gotoChoice(c);
				if (c == 4)
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
			case 1:
				pause();
				break;
			case 2:
				resumeAudio();
				break;
			case 3:
				restart();
				break;
			case 4:
				stop();
                stopped = true;
				break;
			case 5:
				System.out.println("Enter time (" + 0 +
				", " + clip.getMicrosecondLength() + ")");
				Scanner sc = new Scanner(System.in);
				long c1 = sc.nextLong();
				jump(c1);
				break;
            case 6:
				System.out.println(Math.abs(data[(int)clip.getLongFramePosition() * 4]));
				System.out.println(clip.getLongFramePosition() * 4);
				System.out.println(getBytesPositionFromMilliseconds(clip.getMicrosecondPosition(), 2, 2, (int)clip.getFormat().getSampleRate()));
				//System.out.println(clip.getFrameLength());
				System.out.println(data.length);
				break;
                
            
		}
	
	}
	
	// Method to play the audio
	public void play()
	{
		//start the clip
		clip.start();
		//line.start();
		
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
		currentFrame =
		clip.getMicrosecondPosition();
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
		play();
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
		play();
	}
	
	// Method to stop the audio
	public void stop() throws UnsupportedAudioFileException,
	IOException, LineUnavailableException
	{
		currentFrame = 0L;
		//line.stop();
		//line.close();
		input.close();
		output.flush();
		output.flush();
		audioBitch.close();
		audioInputStream.close();
		clip.stop();
		clip.flush();
		clip.drain();
		clip.close();
		line.flush();
		line.drain();
		line.close();
		line.stop();
		source.drain();
		source.flush();
		source.close();
		source.stop();
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
			play();
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

    public static byte [] getAudioDataBytes(byte [] sourceBytes, AudioFormat audioFormat) throws UnsupportedAudioFileException, IllegalArgumentException, Exception {
        if(sourceBytes == null || sourceBytes.length == 0 || audioFormat == null){
            throw new IllegalArgumentException("Illegal Argument passed to this method");
        }
    
        try (final ByteArrayInputStream bais = new ByteArrayInputStream(sourceBytes);
             final AudioInputStream sourceAIS = AudioSystem.getAudioInputStream(bais)) {
            AudioFormat sourceFormat = sourceAIS.getFormat();
            AudioFormat convertFormat = new AudioFormat(AudioFormat.Encoding.PCM_SIGNED, sourceFormat.getSampleRate(), 16, sourceFormat.getChannels(), sourceFormat.getChannels()*2, sourceFormat.getSampleRate(), false);
            try (final AudioInputStream convert1AIS = AudioSystem.getAudioInputStream(convertFormat, sourceAIS);
                 final AudioInputStream convert2AIS = AudioSystem.getAudioInputStream(audioFormat, convert1AIS);
                 final ByteArrayOutputStream baos = new ByteArrayOutputStream()) {
                byte [] buffer = new byte[8192];
                while(true){
                    int readCount = convert2AIS.read(buffer, 0, buffer.length);
                    if(readCount == -1){
                        break;
                    }
                    baos.write(buffer, 0, readCount);
                }
                return baos.toByteArray();
            }
        }
    }

    private static float decodeUnsigned16BitBigEndian(
        final byte[] buffer, final int offset) {
    final byte lower, higher;
    higher = buffer[offset];
    lower = buffer[offset + 1];
    final int sampleInt = ((higher & 0xff) << 8 | lower & 0xff) - 0x8000;
    final float sample = (float) sampleInt / (float) 0x7fff;
    return sample;

		}

    public static float getAmplitude(byte[] data, int len) {
        return (float) (MAX_REPORTABLE_DB + (20 * Math.log10(getRawAmplitude(data, len) / MAX_REPORTABLE_AMP)));
    }

	private static int getRawAmplitude(byte[] data, int len) {
        if (len <= 0 || data == null || data.length <= 0) {
            return 0;
        }

        int sum = 0;
        for (int i = 0; i < len; i++) {
            sum += Math.abs(data[i]);
        }
        return sum / len;
    }

	public static int getFrequency(int k, int samplingrate, int captureSize) {
        return Math.abs((k * samplingrate) / captureSize);
    }
	
	public double returnToSender()
	{
		return Math.abs(data[(int)clip.getLongFramePosition() * 4]);
	}

	public static long getBytesPositionFromMilliseconds(long ms, int bytesPerSample, int numberOfChannels,
	int sampleRate) {
long position = (long) (bytesPerSample * numberOfChannels * ((ms) * sampleRate));
//Dirty hack. Do not ask anything about it.
position -= position % 4;
return position;
}

public static long getBytesPositionFromMilliseconds(long ms, int numberOfChannels) {
return getBytesPositionFromMilliseconds(ms, 2, numberOfChannels, 44100);
}
}




