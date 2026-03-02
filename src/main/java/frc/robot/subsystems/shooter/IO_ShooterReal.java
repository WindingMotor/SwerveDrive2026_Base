// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.RobotConstants;

public class IO_ShooterReal implements IO_ShooterBase {

	private final TalonFX shooterMotorOne;
	private final TalonFX shooterMotorTwo;
	private final VelocityVoltage shooterMotorsRequest;
	private final VoltageOut shooterMotorsVoltageRequest;
	private final VoltageOut turretMotorVoltageRequest;

	private final TalonFX turretMotor;
	private final PositionVoltage turretMotorRequest;

	private final DigitalInput turretHomingSensor = new DigitalInput(9); // DIO 9
	private double slowVolts = RobotConstants.Shooter.TURRET_SLOW_MOVE_VOLTAGE;

	public IO_ShooterReal(
			TalonFXConfiguration shooterMotorOneConfiguration,
			TalonFXConfiguration shooterMotorTwoConfiguration,
			TalonFXConfiguration turretMotorConfiguration) {

		shooterMotorOne =
				new TalonFX(
						RobotConstants.Shooter.SHOOTER_MOTOR_ONE_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		shooterMotorOne.getConfigurator().apply(shooterMotorOneConfiguration);

		shooterMotorTwo =
				new TalonFX(
						RobotConstants.Shooter.SHOOTER_MOTOR_TWO_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		shooterMotorTwo.getConfigurator().apply(shooterMotorTwoConfiguration);

		shooterMotorsRequest = new VelocityVoltage(0.0);
		shooterMotorsVoltageRequest = new VoltageOut(0.0);
		turretMotorVoltageRequest = new VoltageOut(0.0);

		turretMotor =
				new TalonFX(RobotConstants.Shooter.TURRET_MOTOR_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		turretMotor.getConfigurator().apply(turretMotorConfiguration);
		turretMotorRequest = new PositionVoltage(0.0);

		// turretMotor.setPosition(0.0);
	}

	@Override
	public void updateInputs(ShooterInputs inputs) {
		inputs.shooterMotorOneVelocity = shooterMotorOne.getVelocity().getValueAsDouble() * 60;
		inputs.shooterMotorOneTargetVelocity = shooterMotorsRequest.getVelocityMeasure().in(RPM);
		inputs.shooterMotorOneCurrent = shooterMotorOne.getStatorCurrent().getValueAsDouble();

		inputs.shooterMotorTwoVelocity = shooterMotorTwo.getVelocity().getValueAsDouble() * 60;
		inputs.shooterMotorTwoTargetVelocity = shooterMotorsRequest.getVelocityMeasure().in(RPM);
		inputs.shooterMotorTwoCurrent = shooterMotorTwo.getStatorCurrent().getValueAsDouble();

		// Read actual encoder position and convert rotations -> radians for logging
		inputs.turretMotorCurrentPosition =
				turretMotor.getPosition().getValueAsDouble() * RobotConstants.Shooter.ROT_TO_RAD;
		inputs.turretMotorCurrentTargetPosition =
				turretMotorRequest.Position * RobotConstants.Shooter.ROT_TO_RAD;
		inputs.turretMotorCurrent = turretMotor.getStatorCurrent().getValueAsDouble();
		inputs.turretPositionSensor = turretHomingSensor.get();
		inputs.turretVelocity = Math.abs(turretMotor.getVelocity().getValueAsDouble()); // rps
	}

	@Override
	public Pair<StatusCode, StatusCode> setShooterVelocities(double targetRPM) {
		double targetRPS = targetRPM / 60.0; // convert RPM -> rps
		shooterMotorsRequest.withVelocity(targetRPS);
		return Pair.of(
				shooterMotorOne.setControl(shooterMotorsRequest),
				shooterMotorTwo.setControl(shooterMotorsRequest));
	}

	/*@Override
	public StatusCode setTurretPosition(Rotation2d position) {
		double targetRadians = position.getRadians();

		// Clamp to physical turret limits
		if (targetRadians > RobotConstants.Shooter.TURRET_RADIANS_MAX) {
			targetRadians = RobotConstants.Shooter.TURRET_RADIANS_MAX;
		} else if (targetRadians < RobotConstants.Shooter.TURRET_RADIANS_MIN) {
			targetRadians = RobotConstants.Shooter.TURRET_RADIANS_MIN;
		}

		// Convert radians -> turret rotations via shared helper
		double targetRotations = RobotConstants.Shooter.toRotations(targetRadians);
		turretMotorRequest.withPosition(targetRotations);
		return turretMotor.setControl(turretMotorRequest);
	} */

	@Override
	public StatusCode setTurretPosition(double radians) {
		// Clamp to physical turret limits
		double targetRadians =
				Math.max(
						RobotConstants.Shooter.TURRET_RADIANS_MIN,
						Math.min(RobotConstants.Shooter.TURRET_RADIANS_MAX, radians));

		double targetRotations = RobotConstants.Shooter.toRotations(targetRadians);
		turretMotorRequest.withPosition(targetRotations);
		return turretMotor.setControl(turretMotorRequest);
	}

	@Override
	public void setTurretVoltage(double voltage) {
		// FIX: must call setControl() or the motor never receives the command
		turretMotorVoltageRequest.withOutput(voltage);
		turretMotor.setControl(turretMotorVoltageRequest);
	}

	@Override
	public double getTurretPosition() {
		// FIX: read actual encoder position, not the requested target position
		return turretMotor.getPosition().getValueAsDouble() * RobotConstants.Shooter.ROT_TO_RAD;
	}

	@Override
	public double getTurretTargetPosition() {
		// FIX: read actual encoder position, not the requested target position
		return turretMotorRequest.Position * RobotConstants.Shooter.ROT_TO_RAD;
	}

	@Override
	public void setShooterVoltages(double voltages) {
		shooterMotorsVoltageRequest.withOutput(voltages);
		shooterMotorOne.setControl(shooterMotorsVoltageRequest);
		shooterMotorTwo.setControl(shooterMotorsVoltageRequest);
	}

	@Override
	public Boolean homeTurret(Boolean homed) {

		var homingLimit = new CurrentLimitsConfigs();
		homingLimit.StatorCurrentLimit = 10.0; // 10A stator limit
		homingLimit.StatorCurrentLimitEnable = true;
		turretMotor.getConfigurator().apply(homingLimit);

		setTurretVoltage(slowVolts);

		double current = turretMotor.getStatorCurrent().getValueAsDouble();
		double velocity = Math.abs(turretMotor.getVelocity().getValueAsDouble()); // rps

		boolean isStalled = (current > 8.0) && (velocity < 1.0); // Tune these thresholds!

		if (isStalled) {
			slowVolts = slowVolts * -1;
		}

		if (turretHomingSensor.get()) {
			setTurretVoltage(0.0);

			double homeRadiansCenter = 0.0;
			double homeRadians = 0.0 * Math.PI;
			double magnetEdge = 0.04;

			if (slowVolts > 0) {
				homeRadians = homeRadiansCenter - magnetEdge;
			} else {
				homeRadians = homeRadiansCenter + magnetEdge;
			}

			// Convert home angle from radians -> turret rotations
			double homeRotations = homeRadians / RobotConstants.Shooter.ROT_TO_RAD;

			turretMotor.setPosition(homeRotations); // Now in rotations!

			var normalLimit = new CurrentLimitsConfigs();
			normalLimit.StatorCurrentLimit = 30.0; // your normal value
			normalLimit.StatorCurrentLimitEnable = true;
			turretMotor.getConfigurator().apply(normalLimit);
			homed = true;
		}
		return homed;
	}

	@Override
	// DO NOTHING, only used in simulation class.
	public void onShootSimulation() {}
}
