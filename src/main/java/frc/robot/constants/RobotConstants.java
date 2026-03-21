// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class RobotConstants {

	/** Operating mode for the robot (REAL, SIM, or REPLAY). */
	public enum RobotMode {
		REAL,
		SIM,
		REPLAY
	}

	public static final CANBus CANBUS_CANIVORE = new CANBus("canivore", "./logs/canivore.hoot");
	public static final RobotMode ROBOT_MODE = RobotMode.REAL;
	public static final boolean ENABLE_SIM_MANAGER = false;

	// Intake
	public final class Intake {

		public static final int INTAKE_MOTOR_CAN_ID = 9;
		public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			INTAKE_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 55;
			INTAKE_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 1.0;
			INTAKE_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			INTAKE_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		}

		public static final int SLIDER_MOTOR_CAN_ID = 10;
		public static final double INTAKE_MAX_EXTENSION_METERS = 11.5;
		public static final TalonFXConfiguration SLIDER_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			SLIDER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 20;
			SLIDER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 1.0;
			SLIDER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			SLIDER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

			SLIDER_MOTOR_CONFIG.Slot0.kP = 1.9;
			SLIDER_MOTOR_CONFIG.Slot0.kI = 0.1;
			SLIDER_MOTOR_CONFIG.Slot0.kD = 0.0;
			SLIDER_MOTOR_CONFIG.Slot0.kS = 0.35;
			SLIDER_MOTOR_CONFIG.Slot0.kV = 0.3;
			SLIDER_MOTOR_CONFIG.Slot0.kG = -0.155;
		}

		public static final int SENSOR_RIO_ID = 9;
	}

	// Shooter
	public static class Shooter {

		public static final int SHOOTER_MOTOR_ONE_CAN_ID = 21;
		public static final TalonFXConfiguration SHOOTER_MOTOR_ONE_CONFIG = new TalonFXConfiguration();

		static {
			SHOOTER_MOTOR_ONE_CONFIG.CurrentLimits.StatorCurrentLimit = 55;
			SHOOTER_MOTOR_ONE_CONFIG.Feedback.SensorToMechanismRatio = 1.0;
			SHOOTER_MOTOR_ONE_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			SHOOTER_MOTOR_ONE_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kP = 0.2;
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kI = 0.0;
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kD = 0.0;
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kS = 0.232;
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kV = 0.119;
			SHOOTER_MOTOR_ONE_CONFIG.Slot0.kA = 0.0;
		}

		public static final int SHOOTER_MOTOR_TWO_CAN_ID = 20;
		public static final TalonFXConfiguration SHOOTER_MOTOR_TWO_CONFIG = new TalonFXConfiguration();

		static {
			SHOOTER_MOTOR_TWO_CONFIG.CurrentLimits.StatorCurrentLimit = 55;
			SHOOTER_MOTOR_TWO_CONFIG.Feedback.SensorToMechanismRatio = 1.0;
			SHOOTER_MOTOR_TWO_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			SHOOTER_MOTOR_TWO_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kP = 0.2;
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kI = 0.0;
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kD = 0.0;
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kS = 0.232;
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kV = 0.119;
			SHOOTER_MOTOR_TWO_CONFIG.Slot0.kA = 0.0;
		}

		public static final int TURRET_MOTOR_CAN_ID = 22;

		public static final double TURRET_RADIANS_MAX = 0.5;
		public static final double TURRET_RADIANS_MIN = -6.0;
		public static final double TURRET_ANGLE_OFFSET = Math.toRadians(12.0);
		public static final double ROT_TO_RAD = 2.0 * Math.PI;

		public static double toRotations(double radians) {
			return radians / ROT_TO_RAD;
		}

		public static final TalonFXConfiguration TURRET_MOTOR_CONFIG = new TalonFXConfiguration();
		public static final double TURRET_SLOW_MOVE_VOLTAGE = 2.0;

		static {
			TURRET_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 40;

			// SensorToMechanismRatio = 157/11 means Phoenix 6 handles the gear ratio
			// internally. All position and velocity commands are in MECHANISM units
			// (output shaft rotations / rot/s), NOT motor units.
			TURRET_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 157.0 / 11.0;

			TURRET_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			TURRET_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

			TURRET_MOTOR_CONFIG.Slot0.kP = 40.0;
			TURRET_MOTOR_CONFIG.Slot0.kI = 0.5;
			TURRET_MOTOR_CONFIG.Slot0.kD = 0.1;
			TURRET_MOTOR_CONFIG.Slot0.kS = 2.5;

			// ✅ kV MUST be non-zero for .withVelocity() to have any effect.
			// Formula the Kraken uses: FF = kS*sign(v) + kV*v
			// Units: Volts per mechanism rot/s
			// At ~0.5 mech rot/s (≈π rad/s), kV=5.0 gives 2.5V of feedforward
			// on top of kS. Start here and increase by 1.0 until lag shrinks
			// without oscillation at speed.
			TURRET_MOTOR_CONFIG.Slot0.kV = 5.0; // TUNE: start 5.0, increase by 1.0 increments
			TURRET_MOTOR_CONFIG.Slot0.kA = 0.0;

			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
					toRotations(TURRET_RADIANS_MAX);

			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
			TURRET_MOTOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
					toRotations(TURRET_RADIANS_MIN);
		}

		public static final double TURRET_OFFSET_X_METERS = -0.1444752;
		public static final double TURRET_OFFSET_Y_METERS = 0.15875;

		public static final double[][] SHOOTER_RPM_DATA = {
			{2.0, 1700.0},
			{2.5, 1880.0},
			{3.0, 2080.0},
			{3.5, 2150.0},
			{3.7, 2180.0},
			{4.0, 2370.0},
			{4.5, 2530.0},
			{5.0, 2630.0}
		};

		public static final double SHOOTER_RPM_TOLERANCE = 300;

		public static final double SHOOTER_ANGLE_RADIANS = Math.toRadians(67);
		public static final double SHOOTER_WHEEL_DIAMETER_METERS = 0.0762;
		public static final double SHOOTER_EFFICIENCY_FACTOR = 0.93;
	}

	// Indexer
	public static class Indexer {

		public static final int SPINNER_MOTOR_CAN_ID = 15;
		public static final TalonFXConfiguration SPINNER_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			SPINNER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 80;
			SPINNER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 1.0;
			SPINNER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			SPINNER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

			SPINNER_MOTOR_CONFIG.Slot0.kP = 1.0;
			SPINNER_MOTOR_CONFIG.Slot0.kI = 0.0;
			SPINNER_MOTOR_CONFIG.Slot0.kD = 0.0;
			SPINNER_MOTOR_CONFIG.Slot0.kS = 0.0;
			SPINNER_MOTOR_CONFIG.Slot0.kV = 0.0;
			SPINNER_MOTOR_CONFIG.Slot0.kA = 0.0;
		}

		public static final int KICKER_MOTOR_CAN_ID = 29;
		public static final TalonFXConfiguration KICKER_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			KICKER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 15;
			KICKER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 1.0;
			KICKER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			KICKER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		}

		public static final int CLIMB_MOTOR_CAN_ID = 33;
		public static final TalonFXConfiguration CLIMB_MOTOR_CONFIG = new TalonFXConfiguration();

		static {
			CLIMB_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 50;
			CLIMB_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 1.0;
			CLIMB_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			CLIMB_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

			CLIMB_MOTOR_CONFIG.Slot0.kP = 1.0;
			CLIMB_MOTOR_CONFIG.Slot0.kI = 0.0;
			CLIMB_MOTOR_CONFIG.Slot0.kD = 0.0;
			CLIMB_MOTOR_CONFIG.Slot0.kS = 0.0;
			CLIMB_MOTOR_CONFIG.Slot0.kV = 0.0;
			CLIMB_MOTOR_CONFIG.Slot0.kA = 0.0;
		}

		public static final int BEAM_BREAK_DIO = 0;
	}
}
