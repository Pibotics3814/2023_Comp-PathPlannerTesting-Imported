package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GyroSwerveDrive extends SubsystemBase {
  private double[] speed = {0.0, 0.0, 0.0, 0.0};
  private double[] angle = {0.0, 0.0, 0.0, 0.0};
  private RobotStates m_RobotStates;

  private SlewRateLimiter joystickSlewLimiterX;
  private SlewRateLimiter joystickSlewLimiterY;
  private SlewRateLimiter joystickSlewLimiterZ;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry   odometry;
  private SwerveDrivePoseEstimator poseEstimator;
  private ADIS16470_IMU gyro;

  public boolean trustVision;

  private SwerveModule[] swerveMod = {
    new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
  };

  public GyroSwerveDrive(RobotStates robotStates, ADIS16470_IMU gyro) {
    m_RobotStates = robotStates;
    joystickSlewLimiterX = new SlewRateLimiter(Constants.JOYSTICK_X_SLEW_RATE);
    joystickSlewLimiterY = new SlewRateLimiter(Constants.JOYSTICK_Y_SLEW_RATE);
    joystickSlewLimiterZ = new SlewRateLimiter(Constants.JOYSTICK_Z_SLEW_RATE);
    this.gyro = gyro;

    kinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254),
       new Translation2d(Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, -Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254),
        new Translation2d(-Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254),
         new Translation2d(-Constants.SWERVE_FRAME_LENGTH / 2.0 * 0.0254, -Constants.SWERVE_FRAME_WIDTH / 2.0 * 0.0254)
    );
    odometry = new SwerveDriveOdometry(
      kinematics,
       Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis())),
        getModulePositions()
    );

    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis())),
       getModulePositions(),
        new Pose2d()
        );

    trustVision = false;

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getSpeeds,
      this::driveUnits,
      new HolonomicPathFollowerConfig(
        new PIDConstants(2.0, 0, 0.1),
        new PIDConstants(2.0, 0, 0.1),
        Constants.MAX_DRIVETRAIN_SPEED * Constants.DRIVE_POSITION_CONVERSION / 60.0,
        0.29,
        new ReplanningConfig()
      ),
      () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() != DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
    );
  }

  //help

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleState());
  }

  @Override
  public void periodic() {
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1000, 1000, Units.degreesToRadians(20)));
    poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
       Rotation2d.fromDegrees(gyro.getAngle(gyro.getYawAxis())),
        getModulePositions()
    );
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  private SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = swerveMod[0].getPosition();
    positions[1] = swerveMod[3].getPosition();
    positions[2] = swerveMod[1].getPosition();
    positions[3] = swerveMod[2].getPosition();
    return positions;
  }

  public void resetOdometry(Pose2d pose){
    for (int i = 0; i < 4; i++) {
      swerveMod[i].resetModule();
    }
    poseEstimator.resetPosition(
      Rotation2d.fromDegrees(-gyro.getAngle(gyro.getYawAxis()) % 360.0),
       getModulePositions(),
        pose
    );
  }

  public void resetGyro(){
    gyro.reset();
    poseEstimator.resetPosition(Rotation2d.fromDegrees(0), getModulePositions(), getPose());
  }

  public void updateVisionPoseEstimator(Pose2d visionEstimate, double timestamp, double distance){
    //ramp measurement trust based on robot distance
    //poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.1 * Math.pow(15, distance), 0.1 * Math.pow(15, distance), Units.degreesToRadians(20)));
    //poseEstimator.addVisionMeasurement(visionEstimate, timestamp);
    SmartDashboard.putNumber("Tag Distance ", distance);
    SmartDashboard.putNumber("Tag correlation ", 0.1 * Math.pow(15, distance));
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  public void alteredGyroDrive(double dX, double dY, double dZ, double gyroAngle){
    dX = -applyDeadzone(dX, Constants.JOYSTICK_X_DEADZONE);
    dY = -applyDeadzone(dY, Constants.JOYSTICK_Y_DEADZONE);
    dZ = -applyDeadzone(dZ, Constants.JOYSTICK_Z_DEADZONE) * 0.5;
    if ((dX != 0.0) || (dY != 0.0) || (dZ != 0.0)) {
      gyroDrive(
         dX * m_RobotStates.driveMultiplier,
         dY * m_RobotStates.driveMultiplier,
         dZ,
          gyroAngle
      );
      m_RobotStates.inFrontOfCubeStation = false;
    } else{
      speed[0] = 0.0;
      speed[1] = 0.0;
      speed[2] = 0.0;
      speed[3] = 0.0;
      setSetpoints();
    }
  }

  public void gyroDrive(double str, double fwd, double rot, double gyroAngle) {
    double angle = gyroAngle;//poseEstimator.getEstimatedPosition().getRotation().getRadians();
    double intermediary = fwd * Math.cos(angle) + str * Math.sin(angle);
    str = -fwd * Math.sin(angle) + str * Math.cos(angle);
    drive(str, intermediary, rot);
    setSetpoints();
  }

  public void drive(double str, double fwd, double rot) {
    if(str != 0.0 || fwd != 0.0 || rot != 0.0) computeSwerveInputs(str, fwd, rot);
    else{
      speed[0] = 0.0;
      speed[1] = 0.0;
      speed[2] = 0.0;
      speed[3] = 0.0;
    }
    setSetpoints();
  }

  public void driveUnits(ChassisSpeeds driveSpeeds) {
    //driveSpeeds = ChassisSpeeds.discretize(driveSpeeds, 0.2); 
    double str = driveSpeeds.vyMetersPerSecond;
    double fwd = driveSpeeds.vxMetersPerSecond;
    double rot = driveSpeeds.omegaRadiansPerSecond;
    double meterSecToRPM = (1 / Constants.DRIVE_POSITION_CONVERSION * 60.0);
    System.out.println(fwd);
    drive(str * meterSecToRPM / Constants.MAX_DRIVETRAIN_SPEED, fwd * meterSecToRPM / Constants.MAX_DRIVETRAIN_SPEED, rot);
  }

  public SwerveModuleState[] getModuleState(){
    SwerveModuleState[] positions = new SwerveModuleState[4];
    positions[0] = swerveMod[0].getState();
    positions[1] = swerveMod[3].getState();
    positions[2] = swerveMod[1].getState();
    positions[3] = swerveMod[2].getState();
    return positions;
  }

  public void drive(double[] inputs) {
    drive(inputs[0], inputs[1], inputs[2]);
  }

  /*
   * Brake system
   */
  public void brakeAngle() {
    angle[0] = -0.25;
    angle[1] = 0.25;
    angle[2] = -0.25;
    angle[3] = 0.25;
    speed[0] = 0.0;
    speed[1] = 0.0;
    speed[2] = 0.0;
    speed[3] = 0.0;
    setSetpoints();
  }

  private double getDeltaAngle(double alpha, double beta) {
    return 1.0 - Math.abs(Math.abs(alpha - beta) % 2.0 - 1.0);
  }

  private void computeSwerveInputs(double str, double fwd, double rot) {
    double a = str - rot * (Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS);
    double b = str + rot * (Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS);
    double c = fwd - rot * (Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS);
    double d = fwd + rot * (Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS);

    speed[1] = Math.sqrt((a * a) + (d * d));
    speed[2] = Math.sqrt((a * a) + (c * c));
    speed[0] = Math.sqrt((b * b) + (d * d));
    speed[3] = Math.sqrt((b * b) + (c * c));

    angle[1] = Math.atan2(a, d) / Math.PI;
    angle[2] = Math.atan2(a, c) / Math.PI;
    angle[0] = Math.atan2(b, d) / Math.PI;
    angle[3] = Math.atan2(b, c) / Math.PI;
  }

  private void setSetpoints() {
    for (int i = 0; i < 4; i++) {
      double steerAngle = swerveMod[i].getSteerAngle();
      if (getDeltaAngle(angle[i], steerAngle) > 0.5) {
        angle[i] = Math.abs(Math.abs(angle[i] + 2.0) % 2.0) - 1.0;
        speed[i] = -speed[i];
      }
      swerveMod[i].drive(speed[i], angle[i]);
    }
  }

  public void WheelToCoast() {
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void WheelToBrake() {
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  public void outputEncoderPos() {
    SmartDashboard.putNumber("Module 1 encoder", swerveMod[0].getSteerAngle());
    SmartDashboard.putNumber("Module 2 encoder", swerveMod[1].getSteerAngle());
    SmartDashboard.putNumber("Module 3 encoder", swerveMod[2].getSteerAngle());
    SmartDashboard.putNumber("Module 4 encoder", swerveMod[3].getSteerAngle());
  }

  public void motorZero(){
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.set(0.0);
      swerveMod[i].steerMotor.set(0.0);
    }
  }
}
