// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.RobotStates;

public class GrabberToggle extends Command {
  /** Creates a new GrabberCommand. */
  Grabber m_grabber;
  RobotStates robotStates;
  Timer autonWait;
  boolean finished;

  public GrabberToggle(Grabber grabber, RobotStates robotStates) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_grabber = grabber;
    this.robotStates = robotStates;
    autonWait = new Timer();
    addRequirements(grabber, robotStates);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autonWait.reset();
    autonWait.stop();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!finished){
      if (m_grabber.clawOpen) m_grabber.GrabberClose();
      else m_grabber.GrabberOpen();
      autonWait.start();
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !robotStates.autonomous || autonWait.get() >= 1.0;
  }
}
