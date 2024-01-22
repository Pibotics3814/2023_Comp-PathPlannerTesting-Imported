package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class GyroSwerveDriveCommand extends Command {
  DoubleSupplier dX, dY, dZ;
  double headingCorrection;
  IntSupplier povHat;
  boolean driveHeading;
  ADIS16470_IMU m_gyro;
  GyroSwerveDrive m_gyroSwerveDrive;

  PIDController turnController = new PIDController(0.04, 0.1, 0.0);

  public GyroSwerveDriveCommand(
      DoubleSupplier dX,
      DoubleSupplier dY,
      DoubleSupplier dZ,
      IntSupplier povHat,
      ADIS16470_IMU imu,
      GyroSwerveDrive gyroSwerveDrive) {
    this.dX = dX;
    this.dY = dY;
    this.dZ = dZ;
    this.povHat = povHat;
    m_gyro = imu;
    m_gyroSwerveDrive = gyroSwerveDrive;

    turnController.reset();
    turnController.setIntegratorRange(-0.2, 0.2);
    turnController.enableContinuousInput(0.0, 360.0);
    turnController.setTolerance(0.05);

    addRequirements(m_gyroSwerveDrive);
  }

  @Override
  public void execute() {
    driveHeading = povHat.getAsInt() != -1;
    if(driveHeading){
      headingCorrection = MathUtil.clamp(turnController.calculate(360.0 - (m_gyro.getAngle(m_gyro.getYawAxis()) % 360.0), povHat.getAsInt()), -1.0, 1.0);
    }
    m_gyroSwerveDrive.alteredGyroDrive(
        dX.getAsDouble(),
          dY.getAsDouble(),
            driveHeading ? headingCorrection : dZ.getAsDouble(),
              Math.toRadians(m_gyro.getAngle(m_gyro.getYawAxis()))
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
