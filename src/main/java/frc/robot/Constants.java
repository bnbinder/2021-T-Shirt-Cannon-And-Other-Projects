// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

    public static class DRIVE
    {
        public static int topDriveRightCANID = 0;
        public static int topTurnRightCANID = 0;
        public static int bottomDriveRightCANID = 0;
        public static int bottomTurnRightCANID = 0;
        
        public static int topDriveLeftCANID = 0;
        public static int topTurnLeftCANID = 0;
        public static int bottomDriveLeftCANID = 0;
        public static int bottomTurnLeftCANID = 0;

        public static double driveKP = 0.0000001;
        public static double driveKI = 0;
        public static double driveKD = 0;
        public static double driveKF = 0; //! (%ofMotor x 1023) / maxNativeVelocity
        public static double driveOpenRampRate = 0.5;
        public static double driveCloseRampRate = 0.5;

        public static double oneFullRotation = 1023;
        public static double maxNativeVelocity = 2084; //TODO make sure this is right for talon fx

        public static double peakOutputForward = 1;
        public static double peakOutputReverse = 1;
    }
}
