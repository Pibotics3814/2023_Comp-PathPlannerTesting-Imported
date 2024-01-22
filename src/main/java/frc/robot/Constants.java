// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  ////////////////////////////////////////
  //                 OI                 //
  ////////////////////////////////////////

  public static final double JOYSTICK_X_DEADZONE = 0.2;
  public static final double JOYSTICK_Y_DEADZONE = 0.2;
  public static final double JOYSTICK_Z_DEADZONE = 0.25;
  public static final double JOYSTICK_Z2_DEADZONE = 0.2;
  public static final double JOYSTICK_X_SLEW_RATE = 1.0;
  public static final double JOYSTICK_Y_SLEW_RATE = 1.0;
  public static final double JOYSTICK_Z_SLEW_RATE = 1.0;

  public static final int DRIVE_CONTROLLER_PORT = 2;
  public static final int STEER_CONTROLLER_PORT = 0;
  public static final double SLOW_SPEED = 0.5;
  public static final double FAST_SPEED = 1.0;

  public static final double AUTON_1_DISTANCE = 1.5; //meters
  public static final double AUTON_2_DISTANCE = 1.5; //meters
  public static final double AUTON_BALANCE_DISTANCE = 0.4;

  public static final double[] AUTO_BALANCE_PID = {0.003, 4.0e-5, 2.0e-3}; //6.4e-4

  public static final int LIGHT_RELAY_PORT = 0; //port for Spike controlling lights

  public static final double GRID_X_SETPOINT = 0.5;
  public static final double GRID_Y_SETPOINT = 0.8;
  public static final double RIGHT_SUBSTATION_X = 0.5;
  public static final double LEFT_SUBSTATION_X = -0.5;
  public static final double SUBSTATION_Y = 1.1;

  ////////////////////////////////////////
  //               Swerve               //
  ////////////////////////////////////////

  /*
   * Swerve module motor and encoder ids
   * { Front Right, Back Right, Back Left, Front Left }
   */
  public static final int[] SWERVE_DRIVE_MOTOR_IDS = {10, 11, 12, 13};
  public static final int[] SWERVE_STEER_MOTOR_IDS = {20, 21, 22, 23};
  public static final int[] SWERVE_ENCODER_IDS = {30, 31, 32, 33};

  public static final int swerveModuleNumber = 4;

  public static final double[] SWERVE_SETPOINT_OFFSET = {
    // must be between 0 & 360 degrees
    360 - 273.51, // Front Right
    360 - 81.47, // Rear Right
    360 - 311.75, // Rear Left
    360 - 238.45 // Front Left
  };

  public static final double[][] SWERVE_STEER_PID_CONSTANTS = {
    // kP   kI   kD
    { 1.0, 0.0, 0.016 }, //Front Right
		{ 1.0, 0.0, 0.016 }, //Rear Right
		{ 1.0, 0.0, 0.016 }, //Rear Left
		{ 1.0, 0.0, 0.016 }
  };

  public static double[][] SWERVE_DRIVE_PID_CONSTANTS = { 
		// kP   kI   kD  kIz  kFF  kMn  kMx
		{ 1.0e-4, 1.0e-6, 2.0e-4, 0.0, 1.0e-5, -1.0, 1.0 }, //Front Right
		{ 1.0e-4, 1.0e-6, 2.0e-4, 0.0, 1.0e-5, -1.0, 1.0 }, //Rear Right
		{ 1.0e-4, 1.0e-6, 2.0e-4, 0.0, 1.0e-5, -1.0, 1.0 }, //Rear Left
		{ 1.0e-4, 1.0e-6, 2.0e-4, 0.0, 1.0e-5, -1.0, 1.0 }  //Front Left
	};

  public static final double MAX_DRIVETRAIN_SPEED = 2500;

  public static final double[] TAG_ALIGN_STR_PID = {0.4, 0.0, 0.006};
  public static final double[] TAG_ALIGN_ROT_PID = {0.01, 5.0e-2, 0.002};
  public static final double[] TAG_ALIGN_FWD_PID = {0.4, 0.0, 0.006};

  public static final boolean[] STEER_MOTOR_INVERTED = {false, false, false, false};
  public static final boolean[] DRIVE_MOTOR_INVERTED = {false, false, false, false};

  /*
   * Swerve constants for swerve module calculations
   */
  public static final double SWERVE_FRAME_LENGTH = 27.5;
  public static final double SWERVE_FRAME_WIDTH = 27.5;
  public static final double SWERVE_RADIUS = Math.sqrt(Math.pow(SWERVE_FRAME_LENGTH, 2) + Math.pow(SWERVE_FRAME_WIDTH, 2));
  public static final double SWERVE_PID_TOLERANCE = 2.8e-4;
  public static final double DRIVE_POSITION_CONVERSION = (3.8 * 0.0254 * Math.PI) / (6.75);
  //                                      (wheel circum / (encppr * swerve Ratio)

  ////////////////////////////////////////
  //             Arm & Claw             //
  ////////////////////////////////////////

	/*
	 * Arm Motor IDs
	 */
	public static final int SHOULDER_ID_1 = 40;
	public static final int SHOULDER_ID_2 = 41;
	public static final int EXTEND_ID = 42;

  /*
   * Arm control constants
   */
  public static final double EXTEND_HOME_SPEED = -0.6;
  public static final double[] EXTEND_PID_CONSTANTS = {1.0e-1, 0.0, 0.0, 0.0, 0.0, -1.0, 1.0};
  public static final double[] ARM_ANGLE_PID_CONSTANTS = {4.0, 0.2, 0.016, 1.8};
  public static final double[] ARM_ANGLE_FF_UP =   {0.5, 1.0, 0.13};
  public static final double[] ARM_ANGLE_FF_DOWN = {0.5, 1.0, 0.13};
  public static final int ARM_ENCODER_PORT = 4;

  public static final double SCORE_STRAFE_DISTANCE = 1.0;
  public static final double SCORE_FWD_TIME = 0.0;
  public static final double SCORE_SPEED = 0.0;

  public static final double SCORE_ANGLE_TOP_CONE = 0.625; //Original: 0.615
  public static final double SCORE_ANGLE_TOP_CUBE = 0.58;
  public static final double SCORE_ANGLE_MIDDLE_CONE = 0.56;
  public static final double SCORE_ANGLE_MIDDLE_CUBE = 0.54;
  public static final double SCORE_ANGLE_BOTTOM = 0.42;
  public static final double ANGLE_DEPLOY = 0.43;
  //todo: add logic for lower cube top:0.547; middle:0.500
  public static final double DEPLOY_ANGLE = 0.43;
  public static final double STOW_ANGLE = 0.42;
  public static final double SUBSTATION_ANGLE = 0.595; //original: 0.58
  public static final double SUBSTATION_REV = -79.3;
  public static final double EXTEND_REVS_1 = -13.09;
  public static final double EXTEND_REVS_2 = -71.00;
  public static final double EXTEND_REVS_3 = -145.88;
  public static final double EXTEND_REVS_DEPLOY = -74.00;
  public static final double EXTEND_REVS_MID_CUBE = -43.14;

  /*
   * Claw and Arm Pnuematic IDs
   */
  public static final int CLAW_ID_OPEN = 4;
  public static final int CLAW_ID_CLOSE = 1;
  public static final int ARM_ID_OPEN = 3;
  public static final int ARM_ID_CLOSE = 2;
}
