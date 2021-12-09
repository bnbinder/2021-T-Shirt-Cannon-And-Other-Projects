package frc.robot;

import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants;

public class MkUtil {

  public static double nativeToInches(double nativeUnits) {
    return (nativeUnits / 2048.0) * DRIVE.kWheelCircumference;
  }

  public static double inchesToNative(double in) {
    return (in / DRIVE.kWheelCircumference) * 2048.0;
  }

  public static double nativePer100MstoInchesPerSec(double vel) {
    return 10 * nativeToInches(vel);
  }

  public static double inchesPerSecToUnitsPer100Ms(double vel) {
    return inchesToNative(vel) / 10;
  }

  public static double inchesToMeters(double inches) {
    return Units.inchesToMeters(inches);
  }

  public static double nativeToMeters(double nativeUnits) {
    return inchesToMeters(nativeToInches(nativeUnits));
  }

  public static double nativePer100MsToMetersPerSec(double nativeUnits) {
    return inchesToMeters(nativePer100MstoInchesPerSec(nativeUnits));
  }

  public static double metersToInches(double meters) {
    return Units.metersToInches(meters);
  }

  public static double metersPerSecondToNativeUnitsPer100Ms(double meters) {
    return inchesPerSecToUnitsPer100Ms(metersToInches(meters));
  }

  public static DriveSignal cheesyDrive(double throttle, double wheel, boolean cubeInputs) {
    double kThrottleDeadband = 0.0;
    double kWheelDeadband = 0.003;
    double leftMotorSpeed;
    double rightMotorSpeed;
    double moveValue = limitAbsolute(throttle, 1.0);
    double rotateValue = limitAbsolute(wheel, 1.0);
    moveValue = deadband(moveValue, kThrottleDeadband);
    rotateValue = deadband(rotateValue, kWheelDeadband);
    if (cubeInputs) {
      rotateValue = rotateValue * rotateValue * rotateValue;
    }
    rotateValue = rotateValue / 2.3;
    if (moveValue > 0.0) {
      if (rotateValue > 0.0) {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = Math.max(moveValue, rotateValue);
      } else {
        leftMotorSpeed = Math.max(moveValue, -rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      }
    } else {
      if (rotateValue > 0.0) {
        leftMotorSpeed = -Math.max(-moveValue, rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      } else {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
      }
    }
    return new DriveSignal(leftMotorSpeed, rightMotorSpeed);
  }

  public static double limit(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    } else {
      return value;
    }
  }

  private static double deadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public static double limitAbsolute(double a, double max) {
    return Math.abs(a) < max ? a : Math.copySign(max, a);
  }

  public static class DriveSignal {

    public static DriveSignal STOP = new DriveSignal(0, 0);
    protected double mLeftMotor;
    protected double mRightMotor;

    public DriveSignal(double left, double right) {
      mLeftMotor = left;
      mRightMotor = right;
    }

    public double getLeft() {
      return mLeftMotor;
    }

    public double getRight() {
      return mRightMotor;
    }

    @Override
    public String toString() {
      return "L: " + mLeftMotor + " R: " + mRightMotor;
    }
  }

  //! this is ben binders mkutil extension, above is swerdlow code
  //i hope this works
  public static double nativeToDegrees(double gimmeRots, double greerRatio)
  {
    return (gimmeRots * 360) / (greerRatio * Constants.oneEncoderRotation);
  }

  public static double degreesToNative(double gimmeDeg, double greerRatio)
  {
    //! need to see if greer ratio affects degrees as it affects native
    return (gimmeDeg * Constants.oneEncoderRotation * greerRatio) / 360;
  }

  public static double degreesToRadian(double degrees)
  {
    return degrees * (Constants.kPi/180);
  }

  //idk how to convert bool to double and vice versa
  public static boolean doubleToBoolean(double bool)
  {
    if(bool == 1.0)
    {
      return true;
    }
    else if(bool == 0.0)
    {
      return false;
    }
    else
    {
      return false;
    }
  }
}
