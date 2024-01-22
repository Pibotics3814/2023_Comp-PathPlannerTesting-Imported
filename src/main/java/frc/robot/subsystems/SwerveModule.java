package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
	public CANSparkMax            driveMotor;
	private SparkPIDController driveVelocityPIDController;

	public CANSparkMax            steerMotor;
	private CoreCANcoder              steerAngleEncoder;
	private PIDController         steerAnglePIDController;

	public double                 position;
	private int 				  index;
	public RelativeEncoder 		  driveEncoder;
	
	/* the SwerveModule subsystem */
	public SwerveModule( int swerveModIndex ) {
		driveMotor = new CANSparkMax( Constants.SWERVE_DRIVE_MOTOR_IDS[ swerveModIndex ], MotorType.kBrushless );
		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setInverted( Constants.DRIVE_MOTOR_INVERTED[swerveModIndex] );
		driveMotor.setOpenLoopRampRate( 0.2 );
		driveMotor.setSmartCurrentLimit(50, 40);

		driveVelocityPIDController = driveMotor.getPIDController();
		driveVelocityPIDController.setP(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][0]);
		driveVelocityPIDController.setI(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][1]);
		driveVelocityPIDController.setD(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][2]);
		driveVelocityPIDController.setIZone(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][3]);
		driveVelocityPIDController.setFF(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][4]);

		driveEncoder = driveMotor.getEncoder();
		
		steerMotor = new CANSparkMax( Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless );
		steerMotor.setIdleMode(IdleMode.kBrake);
		steerMotor.setInverted( Constants.STEER_MOTOR_INVERTED[swerveModIndex] );
		steerMotor.setSmartCurrentLimit(50, 40);

		steerAngleEncoder = new CoreCANcoder( Constants.SWERVE_ENCODER_IDS[swerveModIndex] );

		steerAnglePIDController = new PIDController( 
			Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][0],
				Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][1],
					Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][2]
		);

        // Limit the PID Controller's input range between -1.0 and 1.0 and set the input
		// to be continuous.
        steerAnglePIDController.enableContinuousInput( -1.0, 1.0 );
		steerAnglePIDController.setTolerance( Constants.SWERVE_PID_TOLERANCE );

		index = swerveModIndex;
	}

	private static final double INVERSE_180 = 1.0 / 180.0; 

	private double getOffsetSteerEncoderAngle(double angle) {
		return (Math.abs(angle + Constants.SWERVE_SETPOINT_OFFSET[index]) % 360.0 - 180.0) * INVERSE_180;
	}

	public double getSteerAngle() {
		return getOffsetSteerEncoderAngle(steerAngleEncoder.getAbsolutePosition().getValue());
	}

	public SwerveModuleState getState(){
		return new SwerveModuleState(driveEncoder.getVelocity() * Constants.DRIVE_POSITION_CONVERSION / 60.0, Rotation2d.fromDegrees(getSteerAngle() * 180.0));
	}

	public void resetModule(){
		driveEncoder.setPosition(0.0);
	}

	public SwerveModulePosition getPosition(){
		return new SwerveModulePosition(driveEncoder.getPosition() * Constants.DRIVE_POSITION_CONVERSION, Rotation2d.fromDegrees(getSteerAngle() * 180.0));
	}

	// angle and speed should be from -1.0 to 1.0, like a joystick input
	public void drive( double speed, double angle ) {
	    // Calculate the turning motor output from the turning PID controller.

		//* Delete slash for tuning offset
		double turnOutput = steerAnglePIDController.calculate( getSteerAngle(), angle );
		steerMotor.set( MathUtil.clamp( turnOutput, -1.0, 1.0 ) );
		driveVelocityPIDController.setReference(Constants.MAX_DRIVETRAIN_SPEED * MathUtil.clamp(speed, -1.0, 1.0), CANSparkMax.ControlType.kVelocity);
		//SmartDashboard.putNumber("Module drive" + index, speed);
		//SmartDashboard.putNumber("Module steer" + index, turnOutput);
		if(Math.abs(speed) <= 0.001) driveMotor.set(0.0);
		//*/SmartDashboard.putNumber("Angle Module " + index, steerAngleEncoder.getAbsolutePosition());
	}

  public void initDefaultCommand() {
    // NOTE: no default command unless running swerve modules seperately
  }
}
