// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

public class StowAngle extends Command {
  /** Creates a new ScoreTop. */
  Arm m_Arm;
  Grabber m_Grabber;
  boolean finished;
  public StowAngle(Arm arm, Grabber grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    m_Grabber = grabber;
    addRequirements(arm, grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    m_Arm.brake = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.ArmDistance(0);
    if(m_Arm.extendAtPos) m_Arm.ArmAngle(Constants.STOW_ANGLE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Arm.shoulderAtPos && m_Arm.extendAtPos;
  }
}
