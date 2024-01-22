// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonWithBalance extends SequentialCommandGroup {
  /** Creates a new AutonWithBalance. */
  public AutonWithBalance(GyroSwerveDrive drivetrain, RobotStates robotStates, Grabber grabber, Arm arm, Limelight limelight, ADIS16470_IMU gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GyroReset(gyro, drivetrain),
      new ScoreTop(arm, grabber, robotStates),
      new PositionApriltag(drivetrain, limelight, robotStates, gyro, 0.2, -0.82, 0.0),
      //new ScoreTop(arm, grabber, robotStates),
      new GrabberToggle(grabber, robotStates),
      //new WaitCommand(1.0),
      new AutonPositionAndStow(drivetrain, gyro, arm, grabber, () -> -Constants.AUTON_BALANCE_DISTANCE,  () -> 0.0),
      new DriveTillAngleChange(drivetrain, gyro),
      new AutoBalance(drivetrain, gyro)
    );
  }
}
