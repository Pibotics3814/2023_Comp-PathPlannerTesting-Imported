// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.RobotStates;

public class ScoreMiddle extends Command {
  /** Creates a new ScoreTop. */
  Arm m_Arm;
  Grabber m_Grabber;
  RobotStates m_robotStates;
  boolean finished;
  public ScoreMiddle(Arm arm, Grabber grabber, RobotStates robotStates) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    m_Grabber = grabber;
    m_robotStates = robotStates;
    addRequirements(arm, grabber, robotStates);
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
    if(m_robotStates.inFrontOfCubeStation) m_Arm.ArmAngle(Constants.SCORE_ANGLE_MIDDLE_CUBE);
    else m_Arm.ArmAngle(Constants.SCORE_ANGLE_MIDDLE_CONE);
    
    if(!finished) finished = m_Arm.shoulderAtPos;
    if(finished) {
      if(m_robotStates.inFrontOfCubeStation) m_Arm.ArmDistance(Constants.EXTEND_REVS_MID_CUBE);
      else m_Arm.ArmDistance(Constants.EXTEND_REVS_1);
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
