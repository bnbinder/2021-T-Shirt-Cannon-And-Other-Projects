// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

    public static double kPi = 3.14159265359;
    public static double oneEncoderRotation = 2048;

    public static class DRIVE
    {
        public static double greerRatio = 6.75;

        public static int topDriveRightCANID = 12;
        public static int bottomDriveRightCANID = 3;
        public static int topDriveLeftCANID = 9;
        public static int bottomDriveLeftCANID = 6;
        
        public static double driveKP = 0.0000001;
        public static double driveKI = 0;
        public static double driveKD = 0;
        public static double driveKF = 0; //! (%ofMotor x 1023) / maxNativeVelocity
        public static double driveOpenRampRate = 0;
        public static double driveCloseRampRate = 0;

        public static double maxNativeVelocity = 2084; //TODO make sure this is right for talon fx

        public static double peakOutputForward = 1;
        public static double peakOutputReverse = 1;

        public static double magicVel = 1095;
        public static double magicAccel = 674;

        public static double kWheelDiameterInches = 5.9575;
        public static double kWheelCircumference = kWheelDiameterInches * kPi;
    }

    public static class TURN
    {
        public static double maxVel = 300;
        public static double maxAccel = 110;

        public static double greerRatio = 12.8;

        public static int topTurnLeftCANID = 4; //top when standing behind elec trical board
        public static int topTurnRightCANID = 7;
        public static int bottomTurnLeftCANID = 2;
        public static int bottomTurnRightCANID = 1;

        public static double turnKP = 0.09;
        public static double turnKI = 0;
        public static double turnKD = 0;
        public static double turnKF = 0;
        public static double turnOpenRampRate = 1;
        public static double turnCloseRampRate = 1;
    }
}
