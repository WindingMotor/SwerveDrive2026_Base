// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

/**
 * This file contains all configuration for the swerve drivetrain including: - PID gains for drive
 * and steer motors - Physical dimensions and gear ratios - Module positions and encoder offsets -
 * CAN device IDs
 *
 * <p><b>RECONFIGURATION GUIDE:</b> Each season, update these sections in order: 1. CAN Device IDs
 * (motors and encoders) 2. Module Positions (physical layout) 3. Encoder Offsets (from billet wheel
 * alignment) 4. PID Gains (from SysId characterization) 5. Physical Constants (wheel radius, gear
 * ratios)
 */
public class LIB_DriveConstants {

	// ====================================================================
	// SECTION 1: PID GAINS (Tune with SysId)
	// ====================================================================

	/**
	 * Steer motor PID gains. Tune these values using Phoenix Tuner X or SysId for optimal turning
	 * performance.
	 */
	private static final Slot0Configs steerGains =
			new Slot0Configs()
					.withKP(100) // Proportional gain (100)
					.withKI(0.0) // Integral gain (0)
					.withKD(0.5) // Derivative gain (0.5)
					.withKS(0.1) // Static friction feedforward (V) 0.1
					.withKV(1.91) // Velocity feedforward (V/(rot/s)) 1.91
					.withKA(0.0) // Acceleration feedforward (V/(rot/s²)) 0.0
					.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

	/**
	 * Drive motor PID gains. Tune these values using Phoenix Tuner X or SysId for optimal driving
	 * performance.
	 */
	private static final Slot0Configs driveGains =
			new Slot0Configs()
					.withKP(1.5) // Proportional gain 2.5
					.withKI(0) // Integral gain
					.withKD(0) // Derivative gain
					.withKS(0) // Static friction feedforward (V)
					.withKV(0.75); // Velocity feedforward (V/(rot/s))0.75

	// ====================================================================
	// SECTION 2: CONTROL CONFIGURATION
	// ====================================================================

	private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
	private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
	private static final DriveMotorArrangement kDriveMotorType =
			DriveMotorArrangement.TalonFX_Integrated;
	private static final SteerMotorArrangement kSteerMotorType =
			SteerMotorArrangement.TalonFX_Integrated;

	// ====================================================================
	// SECTION 3: PHYSICAL CONSTANTS (Measure carefully)
	// ====================================================================

	public static final LinearVelocity kSpeedAt12Volts =
			MetersPerSecond.of(5.0); // Theoretical max speed (m/s)
	public static final double kDriveGearRatio = 6.12; // Drive motor gear ratio
	public static final double kSteerGearRatio = (150.0 / 7.0); // Steer motor gear ratio
	private static final double kCoupleRatio = 3.57; // Azimuth coupling ratio
	private static final Distance kWheelRadius = Inches.of(2.1); // Effective wheel radius (in)
	private static final Current kSlipCurrent = Amps.of(120.0); // Wheel slip current threshold (A)

	// ====================================================================
	// SECTION 4: MOTOR INVERSIONS (Set for correct rotation direction)
	// ====================================================================

	private static final boolean kInvertLeftSide = true; // Invert left side drive motors
	private static final boolean kInvertRightSide = true; // Invert right side drive motors

	// ====================================================================
	// SECTION 5: CAN CONFIGURATION
	// ====================================================================

	private static final int kPigeonId = 13; // Pigeon 2 IMU CAN ID

	// ====================================================================
	// SECTION 6: MOTOR CURRENT LIMITS
	// ====================================================================

	private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
	private static final TalonFXConfiguration steerInitialConfigs =
			new TalonFXConfiguration()
					.withCurrentLimits(
							new CurrentLimitsConfigs()
									.withStatorCurrentLimit(Amps.of(60)) // Steer current limit (A)
									.withStatorCurrentLimitEnable(true));
	private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
	private static final Pigeon2Configuration pigeonConfigs = null;

	// ====================================================================
	// SECTION 7: SIMULATION CONSTANTS (Only used in sim mode)
	// ====================================================================

	private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
	private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
	private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
	private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

	// ====================================================================
	// SECTION 8: MODULE CAN IDs (Update for each robot)
	// ====================================================================

	// Back Left Module 2
	private static final int kBackLeftDriveMotorId = 1; // Drive motor CAN ID 5
	private static final int kBackLeftSteerMotorId = 2; // Steer motor CAN ID 6
	private static final int kBackLeftEncoderId = 20; // CANcoder CAN ID 22
	private static final Angle kBackLeftEncoderOffset = Radians.of(1.732); // Encoder offset (rad)
	private static final boolean kBackLeftSteerMotorInverted = true; // Steer motor inversion
	private static final boolean kBackLeftEncoderInverted = false; // Encoder inversion

	// Back Right Module 3
	private static final int kBackRightDriveMotorId = 3; // Drive motor CAN ID 3
	private static final int kBackRightSteerMotorId = 4; // Steer motor CAN ID 4
	private static final int kBackRightEncoderId = 21; // CANcoder CAN ID 21
	private static final Angle kBackRightEncoderOffset = Radians.of(1.539); // Encoder offset (rad)
	private static final boolean kBackRightSteerMotorInverted = true; // Steer motor inversion
	private static final boolean kBackRightEncoderInverted = false; // Encoder inversion

	// Front Right Module 1
	private static final int kFrontRightDriveMotorId = 5; // Drive motor CAN ID 1
	private static final int kFrontRightSteerMotorId = 6; // Steer motor CAN ID 2
	private static final int kFrontRightEncoderId = 22; // CANcoder CAN ID 20
	private static final Angle kFrontRightEncoderOffset = Radians.of(2.416); // Encoder offset (rad)
	private static final boolean kFrontRightSteerMotorInverted = true; // Steer motor inversion
	private static final boolean kFrontRightEncoderInverted = false; // Encoder inversion

	// Front Left Module 0
	private static final int kFrontLeftDriveMotorId = 7; // Drive motor CAN ID 7
	private static final int kFrontLeftSteerMotorId = 8; // Steer motor CAN ID 8
	private static final int kFrontLeftEncoderId = 23; // CANcoder CAN ID 23
	private static final Angle kFrontLeftEncoderOffset = Radians.of(-1.532); // Encoder offset (rad)
	private static final boolean kFrontLeftSteerMotorInverted = true; // Steer motor inversion
	private static final boolean kFrontLeftEncoderInverted = false; // Encoder inversion

	// ====================================================================
	// SECTION 9: MODULE POSITIONS (Measure from robot center) (keep sign same)
	// ====================================================================

	// Front Left Position
	private static final Distance kFrontLeftXPos = Inches.of(10.375); // X position from center (in)
	private static final Distance kFrontLeftYPos = Inches.of(11.375); // Y position from center (in)

	// Front Right Position
	private static final Distance kFrontRightXPos = Inches.of(10.375); // X position from center (in)
	private static final Distance kFrontRightYPos = Inches.of(-11.375); // Y position from center (in)

	// Back Left Position
	private static final Distance kBackLeftXPos = Inches.of(-10.375); // X position from center (in)
	private static final Distance kBackLeftYPos = Inches.of(11.375); // Y position from center (in)

	// Back Right Position
	private static final Distance kBackRightXPos = Inches.of(-10.375); // X position from center (in)
	private static final Distance kBackRightYPos = Inches.of(-11.375); // Y position from center (in)

	// ====================================================================
	// SECTION 10: PATHPLANNER CONFIG
	// ====================================================================

	public static final double ROBOT_MASS_KG = 64.41;
	public static final double ROBOT_MOI = 6.883;
	public static final double WHEEL_COF = 1.2;

	// ====================================================================
	// SECTION 11: DRIVETRAIN CONSTRUCTION (Do not modify)
	// ====================================================================

	public static final SwerveDrivetrainConstants DrivetrainConstants =
			new SwerveDrivetrainConstants()
					.withCANBusName(RobotConstants.CANBUS_CANIVORE.getName())
					.withPigeon2Id(kPigeonId)
					.withPigeon2Configs(pigeonConfigs);

	private static final SwerveModuleConstantsFactory<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			ConstantCreator =
					new SwerveModuleConstantsFactory<
									TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
							.withDriveMotorGearRatio(kDriveGearRatio)
							.withSteerMotorGearRatio(kSteerGearRatio)
							.withCouplingGearRatio(kCoupleRatio)
							.withWheelRadius(kWheelRadius)
							.withSteerMotorGains(steerGains)
							.withDriveMotorGains(driveGains)
							.withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
							.withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
							.withSlipCurrent(kSlipCurrent)
							.withSpeedAt12Volts(kSpeedAt12Volts)
							.withDriveMotorType(kDriveMotorType)
							.withSteerMotorType(kSteerMotorType)
							.withDriveMotorInitialConfigs(driveInitialConfigs)
							.withSteerMotorInitialConfigs(steerInitialConfigs)
							.withEncoderInitialConfigs(encoderInitialConfigs)
							.withSteerInertia(kSteerInertia)
							.withDriveInertia(kDriveInertia)
							.withSteerFrictionVoltage(kSteerFrictionVoltage)
							.withDriveFrictionVoltage(kDriveFrictionVoltage);

	public static final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			FrontLeft =
					ConstantCreator.createModuleConstants(
							kFrontLeftSteerMotorId,
							kFrontLeftDriveMotorId,
							kFrontLeftEncoderId,
							kFrontLeftEncoderOffset,
							kFrontLeftXPos,
							kFrontLeftYPos,
							kInvertLeftSide,
							kFrontLeftSteerMotorInverted,
							kFrontLeftEncoderInverted);

	public static final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			FrontRight =
					ConstantCreator.createModuleConstants(
							kFrontRightSteerMotorId,
							kFrontRightDriveMotorId,
							kFrontRightEncoderId,
							kFrontRightEncoderOffset,
							kFrontRightXPos,
							kFrontRightYPos,
							kInvertRightSide,
							kFrontRightSteerMotorInverted,
							kFrontRightEncoderInverted);

	public static final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			BackLeft =
					ConstantCreator.createModuleConstants(
							kBackLeftSteerMotorId,
							kBackLeftDriveMotorId,
							kBackLeftEncoderId,
							kBackLeftEncoderOffset,
							kBackLeftXPos,
							kBackLeftYPos,
							kInvertLeftSide,
							kBackLeftSteerMotorInverted,
							kBackLeftEncoderInverted);

	public static final SwerveModuleConstants<
					TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
			BackRight =
					ConstantCreator.createModuleConstants(
							kBackRightSteerMotorId,
							kBackRightDriveMotorId,
							kBackRightEncoderId,
							kBackRightEncoderOffset,
							kBackRightXPos,
							kBackRightYPos,
							kInvertRightSide,
							kBackRightSteerMotorInverted,
							kBackRightEncoderInverted);

	// ====================================================================
	// SECTION 12: SWERVE DRIVETRAIN CLASS
	// ====================================================================

	/**
	 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
	 *
	 * <p>Extends SwerveDrivetrain to provide TalonFX and CANcoder device types. This class handles
	 * construction of hardware devices automatically.
	 */
	public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

		public TunerSwerveDrivetrain(
				SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
			super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
		}

		public TunerSwerveDrivetrain(
				SwerveDrivetrainConstants drivetrainConstants,
				double odometryUpdateFrequency,
				SwerveModuleConstants<?, ?, ?>... modules) {
			super(
					TalonFX::new,
					TalonFX::new,
					CANcoder::new,
					drivetrainConstants,
					odometryUpdateFrequency,
					modules);
		}

		public TunerSwerveDrivetrain(
				SwerveDrivetrainConstants drivetrainConstants,
				double odometryUpdateFrequency,
				Matrix<N3, N1> odometryStandardDeviation,
				Matrix<N3, N1> visionStandardDeviation,
				SwerveModuleConstants<?, ?, ?>... modules) {
			super(
					TalonFX::new,
					TalonFX::new,
					CANcoder::new,
					drivetrainConstants,
					odometryUpdateFrequency,
					odometryStandardDeviation,
					visionStandardDeviation,
					modules);
		}
	}
}
