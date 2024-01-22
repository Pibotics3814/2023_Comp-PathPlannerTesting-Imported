// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GyroSwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonPositionAndStow extends ParallelDeadlineGroup {
  /** Creates a new AutonPositionAndStow. */
  public AutonPositionAndStow(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro, Arm arm, Grabber grabber, DoubleSupplier distance, DoubleSupplier direction) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new AutoDriveDistance(drivetrain, gyro, distance, direction));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new StowAngle(arm, grabber));
  }
}
