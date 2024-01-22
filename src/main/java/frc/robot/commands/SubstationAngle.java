// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

public class SubstationAngle extends Command {
  /** Creates a new ScoreTop. */
  Arm m_Arm;
  Grabber m_Grabber;
  boolean finished1;
  boolean finished2;
  boolean finished3;
  public SubstationAngle(Arm arm, Grabber grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = arm;
    m_Grabber = grabber;
    addRequirements(arm, grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished1 = false;
    finished2 = false;
    finished3 = false;
    m_Arm.brake = false;
    DriverStation.reportError("init Sub", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!finished1) {
      m_Arm.ArmDistance(-2);
      finished1 = m_Arm.extendAtPos;
      DriverStation.reportError("home arm", false);
      DriverStation.reportError("extend state " + finished1, false);
    }

    if(finished1){
      m_Arm.ArmAngle(Constants.SUBSTATION_ANGLE);
      finished2 = m_Arm.shoulderAtPos;
      DriverStation.reportError("angle arm", false);
    }

    if(finished1 && finished2){
      m_Arm.ArmDistance(Constants.SUBSTATION_REV);
      finished3 = m_Arm.shoulderAtPos && m_Arm.extendAtPos;
      DriverStation.reportError("final extend arm", false);
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
