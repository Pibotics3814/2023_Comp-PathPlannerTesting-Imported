// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotStates;

public class PositionApriltag extends Command {

  private GyroSwerveDrive drivetrain;
  private Limelight limelight;
  private ADIS16470_IMU gyro;
  private RobotStates robotStates;

  private boolean start;

  private int loopNum;

  private PIDController strController = new PIDController(Constants.TAG_ALIGN_STR_PID[0], Constants.TAG_ALIGN_STR_PID[1], Constants.TAG_ALIGN_STR_PID[2]);
  private PIDController fwdController = new PIDController(Constants.TAG_ALIGN_FWD_PID[0], Constants.TAG_ALIGN_FWD_PID[1], Constants.TAG_ALIGN_FWD_PID[2]);
  private PIDController rotController = new PIDController(Constants.TAG_ALIGN_ROT_PID[0], Constants.TAG_ALIGN_ROT_PID[1], Constants.TAG_ALIGN_ROT_PID[2]);

  /** Creates a new AutoPosition. */
  public PositionApriltag(GyroSwerveDrive drivetrain, Limelight limelight, RobotStates robotStates, ADIS16470_IMU gyro, double goalX, double goalY, double goalZ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.robotStates = robotStates;
    this.gyro = gyro;

    strController.reset();
    fwdController.reset();
    rotController.reset();

    strController.setIntegratorRange(-0.2, 0.2);
    fwdController.setIntegratorRange(-0.2, 0.2);
    rotController.setIntegratorRange(-0.2, 0.2);

    strController.setTolerance(0.02);
    fwdController.setTolerance(0.02);
    rotController.setTolerance(0.05);

    rotController.enableContinuousInput(0.0, 360.0);

    strController.setSetpoint(goalX);
    fwdController.setSetpoint(goalY);
    rotController.setSetpoint(goalZ);

    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    strController.reset();
    fwdController.reset();
    rotController.reset();
    robotStates.tracking = true;
    start = false;
    loopNum = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double correctionX = 0.0;
    double correctionY = 0.0;
    double correctionZ = 0.0;

    if(limelight.targetInView){
      drivetrain.resetOdometry(limelight.targetPose2d);
      start = true;
    }

    if(start){
      if(!strController.atSetpoint()) correctionX = MathUtil.clamp(strController.calculate(drivetrain.getPose().getY()), -0.3, 0.3);
      if(!fwdController.atSetpoint()) correctionY = MathUtil.clamp(fwdController.calculate(drivetrain.getPose().getX()), -0.3, 0.3);
      if(!rotController.atSetpoint()) correctionZ = MathUtil.clamp(rotController.calculate(gyro.getAngle(gyro.getYawAxis())) % 360.0, -0.4, 0.4);
      drivetrain.drive(correctionX, correctionY, correctionZ);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.motorZero();
    start = false;
    robotStates.tracking = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return strController.atSetpoint() && fwdController.atSetpoint() && rotController.atSetpoint() && robotStates.autonomous;
  }
}
