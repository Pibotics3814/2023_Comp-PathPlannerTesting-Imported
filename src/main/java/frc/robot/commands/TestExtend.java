// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

public class TestExtend extends Command {
  /** Creates a new ScoreTop. */
  Arm m_Arm;
  Grabber m_Grabber;
  boolean finished;
  public TestExtend(Arm arm, Grabber grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    m_Grabber = grabber;
    addRequirements(arm, grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.ArmDistance(Constants.EXTEND_REVS_3);
    if(m_Arm.extendAtPos){
      m_Grabber.GrabberOpen();
      finished = true;
    }
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
