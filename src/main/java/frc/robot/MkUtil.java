package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.TURN;
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

  //!     this is ben binders mkutil extension, above is swerdlow code

  public static double nativeToDegrees(double gimmeRots, double greerRatio)
  {
    return (gimmeRots * 360) / (greerRatio * Constants.oneEncoderRotation);
  }

  public static double degreesToNative(double gimmeDeg, double greerRatio)
  {
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

  public static double isPositive(double number, double numberTwo)
  {
    if(number < 0.0)
    {
      return -1.0 * numberTwo;
    }
    else if(number > 0.0)
    {
      return 1.0 * numberTwo;
    }
    else
    {
      return numberTwo;
    }
  }

  //!     stolen from team 6624
  
    /**
 * Get the closest angle between the given angles.
 */
  public static double closestAngle(double a, double b)
  {
          // get direction
          //!     modulo = %, right?
          //!     has to be since it works lmao
          double dir = (b % 360.0) - (a % 360.0);

          // convert from -360 to 360 to -180 to 180
          if (Math.abs(dir) > 180.0)
          {
                  dir = -(Math.signum(dir) * 360.0) + dir;
          }
          return dir;
  }

  //!     also stolen from 6624
  public static double setDirection(TalonFX talon, double setpoint, double greerRatio)
  {
    // use the fastest way
    double currentAngle = MkUtil.nativeToDegrees(talon.getSelectedSensorPosition(), greerRatio);
    return currentAngle + closestAngle(currentAngle, setpoint);
  }



  //!     stolen from 1684
  /*
  public static void inversionAwarness(TalonFX talon, double wa)
  {
    double encoderw = talon.getSelectedSensorPosition();
    double azimuthAngle = encoderw;
    double azimuthError = azimuthAngle - wa;

    if(Math.abs(azimuthError) > 90)//assuming our angles are in degrees
    {
      azimuthError = azimuthError - 180 * Math.signum(azimuthError);
      talon.setInverted(true);
    }
      else
      {
        talon.setInverted(false);
      }

  }
  */

  public static double setDirection(TalonFX talon, double setpoint, PIDController pid)
    {
        double currentAngle = nativeToDegrees(talon.getSelectedSensorPosition(), TURN.greerRatio);
        // find closest angle to setpoint
        double setpointAngle = closestAngle(currentAngle, setpoint);
        // find closest angle to setpoint + 180
        double setpointAngleFlipped = closestAngle(currentAngle, setpoint + 180.0);
        // if the closest angle to setpoint is shorter
        if (Math.abs(setpointAngle) <= Math.abs(setpointAngleFlipped))
        {
            // unflip the motor direction use the setpoint
            pid.setP(Math.abs(pid.getP()) * 1.0);
            return (currentAngle + setpointAngle);
        }
        // if the closest angle to setpoint + 180 is shorter
        else
        {
            // flip the motor direction and use the setpoint + 180
            pid.setP(Math.abs(pid.getP()) * -1.0);
            return (currentAngle + setpointAngleFlipped);
        }
    }



}
