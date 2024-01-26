// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GyroSwerveDrive;

public class GyroReset extends Command {
  /** Creates a new GyroReset. */
  ADIS16470_IMU gyro;
  GyroSwerveDrive drivetrain;

  public GyroReset(ADIS16470_IMU gyro, GyroSwerveDrive drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.gyro = gyro;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gyro.reset();
    drivetrain.resetOdometry(new Pose2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
