// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;

public class TurnToHeading extends Command {
  /** Creates a new TurnToHeading. */
  GyroSwerveDrive drivetrain;
  ADIS16470_IMU gyro;

  PIDController turnController = new PIDController(
    Constants.TAG_ALIGN_ROT_PID[0],
     Constants.TAG_ALIGN_ROT_PID[1],
      Constants.TAG_ALIGN_ROT_PID[2]
  );
  
  public TurnToHeading(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro, DoubleSupplier heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.gyro = gyro;

    turnController.setSetpoint(heading.getAsDouble());

    turnController.enableContinuousInput(0.0, 360.0);
    turnController.setTolerance(0.5, 1.0);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0.0, 0.0, MathUtil.clamp(turnController.calculate(gyro.getAngle(gyro.getYawAxis()) % 360.0), -0.4, 0.4));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
