// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public boolean targetInView;
  public Pose2d targetPose2d;
  public double closestTagID;
  private RobotStates robotStates;
  private GyroSwerveDrive m_drivetrain;

  public Limelight(RobotStates robotStates, GyroSwerveDrive m_drivetrain) {
    this.robotStates = robotStates;
    this.m_drivetrain = m_drivetrain;
    targetInView = false;
  }

  @Override
  public void periodic() {
    targetInView = LimelightHelpers.getTV("limelight");
    m_drivetrain.trustVision = targetInView;
    if(robotStates.tracking){
      if(targetInView){
        targetPose2d = LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_TargetSpace("limelight"));
        closestTagID = LimelightHelpers.getFiducialID("limelight");

        Pose2d tempPose = LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_wpiBlue("limelight"));
        if(tempPose.getX() != -1000.0){ // Checks if pose is valid
          double distance = Math.sqrt(Math.pow(targetPose2d.getX(), 2.0) + Math.pow(targetPose2d.getY(), 2.0));
          m_drivetrain.updateVisionPoseEstimator(tempPose, Timer.getFPGATimestamp(), distance);
        }
      }
    }
  }
}