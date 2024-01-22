// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  private DoubleSolenoid claw =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM, Constants.CLAW_ID_OPEN, Constants.CLAW_ID_CLOSE);
  public boolean clawOpen;
  /** Creates a new Grabber. */
  public Grabber() {
    clawOpen = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void GrabberOpen() {
    // Opens the grabber
    claw.set(DoubleSolenoid.Value.kForward);
    clawOpen = true;
  }

  public void GrabberClose() {
    // Closes the Grabber
    claw.set(DoubleSolenoid.Value.kReverse);
    clawOpen = false;
  }
}
