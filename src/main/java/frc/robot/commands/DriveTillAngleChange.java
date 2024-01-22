// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GyroSwerveDrive;

public class DriveTillAngleChange extends Command {
  /** Creates a new DriveTillAngleChange. */
  GyroSwerveDrive drivetrain;
  ADIS16470_IMU gyro;
  boolean finished;
  Timer driveTimer;

  public DriveTillAngleChange(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro) {
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    driveTimer = new Timer();
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    drivetrain.drive(0.0, -0.25, 0.0);
    driveTimer.reset();
    driveTimer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(gyro.getXFilteredAccelAngle()) >= 12.0) finished = true;
    if(finished) driveTimer.start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveTimer.get() >= 1.0;
  }
}
