// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive;

public class DriveStraight extends CommandBase {
  private Drive mDrive = Drive.getInstance();
  double j;
  public DriveStraight(double i) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    j = i;
    mDrive.zeroSensors();
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    mDrive.setMagicStraight(j);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    mDrive.updateMagicStraight();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return mDrive.isMagicStraightDone();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {}
}
