// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GyroSwerveDrive;

public class AutoDriveDistance extends Command {
  /** Creates a new AutoDriveDistance. */
  private GyroSwerveDrive drivetrain;
  private ADIS16470_IMU gyro;
  private Timer autoTimer;
  private TrapezoidProfile driveProfile;

  private double direction;

  double rotateSpeed;
  
  public AutoDriveDistance(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro, DoubleSupplier distance, DoubleSupplier direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    this.direction = direction.getAsDouble();

    driveProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(0.3, 0.5),
       new TrapezoidProfile.State(distance.getAsDouble(), 0.0),
        new TrapezoidProfile.State(0.0, 0.0)
    );
    
    autoTimer = new Timer();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoTimer.reset();
    autoTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = driveProfile.calculate(autoTimer.get());;
    drivetrain.drive(setpoint.velocity * Math.sin(Math.toRadians(direction)), setpoint.velocity * Math.cos(Math.toRadians(direction)), 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveProfile.totalTime() + 0.5 <= autoTimer.get();
  }
}
