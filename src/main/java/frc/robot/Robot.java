// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Compressor comp;
  private final Field2d m_field = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    comp = new Compressor(0, PneumaticsModuleType.CTREPCM);
    comp.enableDigital();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.m_robotStates.autonomous = true;

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.m_gyrp.reset();
    m_robotContainer.m_robotStates.autonomous = false;
    m_robotContainer.m_gyrp.reset();
    m_robotContainer.m_gyroSwerveDrive.resetOdometry(new Pose2d());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Gyro Angle", m_robotContainer.m_gyrp.getAngle(m_robotContainer.m_gyrp.getYawAxis()));
    SmartDashboard.putNumber("Gyro X", m_robotContainer.m_gyrp.getXComplementaryAngle() > 180.0 ? -(m_robotContainer.m_gyrp.getXComplementaryAngle() - 360) : m_robotContainer.m_gyrp.getXComplementaryAngle());
    SmartDashboard.putNumber("Gyro Y", m_robotContainer.m_gyrp.getYComplementaryAngle() > 180.0 ? -(m_robotContainer.m_gyrp.getYComplementaryAngle() - 360) : m_robotContainer.m_gyrp.getYComplementaryAngle());
    SmartDashboard.putBoolean("Home Switch", m_robotContainer.m_arm.extendHomeSwitch.get());
    SmartDashboard.putBoolean("Grabber", m_robotContainer.m_grabber.clawOpen);
    SmartDashboard.putBoolean("Valid Target", m_robotContainer.m_Limelight.targetInView);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("Omega", Math.toRadians(m_robotContainer.m_gyrp.getRate(m_robotContainer.m_gyrp.getYawAxis())));
    m_field.setRobotPose(m_robotContainer.m_gyroSwerveDrive.getPose());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
