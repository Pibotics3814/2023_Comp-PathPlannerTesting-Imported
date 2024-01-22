// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

public class DirectArmCommand extends Command {
  /** Creates a new ArmCommand. */
  Arm m_arm;

  DoubleSupplier armAngleSpeed, armExtendSpeed;

  public DirectArmCommand(Arm arm, DoubleSupplier angleSpeed, DoubleSupplier extendSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    armAngleSpeed = angleSpeed;
    armExtendSpeed = extendSpeed;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.ArmDirectControl(armAngleSpeed, armExtendSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
