// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonSRX shoulder1;
  private WPI_TalonSRX shoulder2;
  private CANSparkMax extend;
  private DutyCycleEncoder shoulderEncoder;

  public boolean extendAtPos;
  public boolean shoulderAtPos;

  private DoubleSolenoid armBrake;

  private PIDController angleController;

  public RelativeEncoder extendEncoder;
  private SparkPIDController extendController;

  public Double extendOffset;
  private boolean extendIsHomed;

  ArmFeedforward armFFUp, armFFDown;

  public DigitalInput extendHomeSwitch;

  public boolean brake;

  public Arm() {
    extend = new CANSparkMax(Constants.EXTEND_ID, MotorType.kBrushless);
    extend.setIdleMode(IdleMode.kBrake);
    extend.setSmartCurrentLimit(70, 50);

    armBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ARM_ID_OPEN, Constants.ARM_ID_CLOSE);
    armBrake.set(DoubleSolenoid.Value.kReverse);
    shoulder1 = new WPI_TalonSRX(Constants.SHOULDER_ID_1);
    shoulder2 = new WPI_TalonSRX(Constants.SHOULDER_ID_2);
    shoulder1.setInverted(false);
    shoulder2.setInverted(true);
    shoulder1.setNeutralMode(NeutralMode.Brake);
    shoulder2.setNeutralMode(NeutralMode.Brake);

    shoulder1.configPeakCurrentLimit(70, 1);
    shoulder2.configPeakCurrentLimit(70, 1);

    shoulderEncoder = new DutyCycleEncoder(Constants.ARM_ENCODER_PORT);

    extendEncoder = extend.getEncoder();
    extend.setSmartCurrentLimit(50, 40);

    extendController = extend.getPIDController();
    extendController.setP(Constants.EXTEND_PID_CONSTANTS[0]);
    extendController.setI(Constants.EXTEND_PID_CONSTANTS[1]);
    extendController.setD(Constants.EXTEND_PID_CONSTANTS[2]);
    extendController.setIZone(Constants.EXTEND_PID_CONSTANTS[3]);
    extendController.setFF(Constants.EXTEND_PID_CONSTANTS[4]);
    extendController.setOutputRange(Constants.EXTEND_PID_CONSTANTS[5], Constants.EXTEND_PID_CONSTANTS[6]);

    extendHomeSwitch = new DigitalInput(6);

    extendOffset = 0.0;
    extendIsHomed = false;

    angleController =
        new PIDController(
            Constants.ARM_ANGLE_PID_CONSTANTS[0],
            Constants.ARM_ANGLE_PID_CONSTANTS[1],
            Constants.ARM_ANGLE_PID_CONSTANTS[2]);

    angleController.disableContinuousInput();
    angleController.setTolerance(0.015);

    armFFUp = new ArmFeedforward(Constants.ARM_ANGLE_FF_UP[0], Constants.ARM_ANGLE_FF_UP[1], Constants.ARM_ANGLE_FF_UP[2]);
    armFFDown = new ArmFeedforward(Constants.ARM_ANGLE_FF_DOWN[0], Constants.ARM_ANGLE_FF_DOWN[1], Constants.ARM_ANGLE_FF_DOWN[2]);

    extendAtPos = false;
    shoulderAtPos = false;
    brake = false;
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  public void ArmDirectControl(DoubleSupplier passedArm, DoubleSupplier passedExtend) {
    double armSpeed = applyDeadzone(passedArm.getAsDouble(), Constants.JOYSTICK_X_DEADZONE);
    double extendSpeed = applyDeadzone(passedExtend.getAsDouble(), Constants.JOYSTICK_X_DEADZONE);
    if(!extendHomeSwitch.get() && (extendSpeed <= 0.0)){
      extendSpeed = 0.0;
    }
    if (armSpeed != 0.0) armBrake.set(DoubleSolenoid.Value.kForward);
    else armBrake.set(DoubleSolenoid.Value.kReverse);
    shoulder1.set(armSpeed * 0.5);
    shoulder2.set(armSpeed * 0.5);
    extend.set(-extendSpeed);
  }

  public double GetArmAngle(){
    return shoulderEncoder.getAbsolutePosition();
  }

  public void ArmAngle(double angle) {
    double correction = 0;
    //*
    double trueAngle = GetArmAngle();
    if((trueAngle >= 0.2) && (trueAngle <= 0.75)){
      correction = -angleController.calculate(trueAngle, angle) + (0.53 - angle) * Constants.ARM_ANGLE_PID_CONSTANTS[3];
      //armBrake.set(DoubleSolenoid.Value.kForward);
    }
    //*/
    shoulderAtPos = angleController.atSetpoint();
    if(!brake) brake = Math.abs(angle - trueAngle) <= 0.01;
    //if (Math.abs(angle - trueAngle) <= 0.01){
    if(brake){
      armBrake.set(DoubleSolenoid.Value.kReverse);
    } else {
      armBrake.set(DoubleSolenoid.Value.kForward);
    }

    correction = MathUtil.clamp(correction, -0.5, 0.5);
    shoulder1.set(correction);
    shoulder2.set(correction);
  }

  public void ArmDistance(double position) {
    extendAtPos = false;
    if(!extendIsHomed){
      extend.set(-Constants.EXTEND_HOME_SPEED);
      if(!extendHomeSwitch.get()){
        extend.set(0.0);
        extendOffset = extendEncoder.getPosition();
        extendIsHomed = true;
      }
    } else{
      extendController.setReference(position + extendOffset, ControlType.kPosition);
    }
    extendAtPos = Math.abs(extend.getEncoder().getPosition() - extendOffset - position) <= 10;
  }
}
