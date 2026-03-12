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

	private final DigitalInput turretHomingSensor = new DigitalInput(9);
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
	}

	@Override
	public void updateInputs(ShooterInputs inputs) {
		inputs.shooterMotorOneVelocity = shooterMotorOne.getVelocity().getValueAsDouble() * 60;
		inputs.shooterMotorOneTargetVelocity = shooterMotorsRequest.getVelocityMeasure().in(RPM);
		inputs.shooterMotorOneCurrent = shooterMotorOne.getStatorCurrent().getValueAsDouble();

		inputs.shooterMotorTwoVelocity = shooterMotorTwo.getVelocity().getValueAsDouble() * 60;
		inputs.shooterMotorTwoTargetVelocity = shooterMotorsRequest.getVelocityMeasure().in(RPM);
		inputs.shooterMotorTwoCurrent = shooterMotorTwo.getStatorCurrent().getValueAsDouble();

		inputs.turretMotorCurrentPosition =
				turretMotor.getPosition().getValueAsDouble() * RobotConstants.Shooter.ROT_TO_RAD;
		inputs.turretMotorCurrentTargetPosition =
				turretMotorRequest.Position * RobotConstants.Shooter.ROT_TO_RAD;
		inputs.turretMotorCurrent = turretMotor.getStatorCurrent().getValueAsDouble();
		inputs.turretPositionSensor = turretHomingSensor.get();
		inputs.turretVelocity = Math.abs(turretMotor.getVelocity().getValueAsDouble());
	}

	@Override
	public Pair<StatusCode, StatusCode> setShooterVelocities(double targetRPM) {
		double targetRPS = targetRPM / 60.0;
		shooterMotorsRequest.withVelocity(targetRPS);
		return Pair.of(
				shooterMotorOne.setControl(shooterMotorsRequest),
				shooterMotorTwo.setControl(shooterMotorsRequest));
	}

	/**
	 * Sets the turret position with velocity feedforward.
	 *
	 * <p>SensorToMechanismRatio = 157/11 is already configured on the Kraken, so Phoenix 6 handles
	 * the gear ratio internally. Both position and velocity are in MECHANISM units: - position:
	 * toRotations(radians) → mechanism rotations - velocity: velocityRadPerSec / (2π) → mechanism
	 * rot/s (NO gear ratio needed here)
	 *
	 * <p>The Kraken uses .withVelocity() at 1kHz via: FF = kS*sign(v) + kV*v This fills in the 9 PID
	 * cycles between each 10ms Java update automatically.
	 */
	@Override
	public StatusCode setTurretPosition(double radians, double velocityRadPerSec) {
		// Clamp to physical soft limits
		double targetRadians =
				Math.max(
						RobotConstants.Shooter.TURRET_RADIANS_MIN,
						Math.min(RobotConstants.Shooter.TURRET_RADIANS_MAX, radians));

		// Convert to mechanism rotations — SensorToMechanismRatio handles gear ratio internally
		double targetRotations = RobotConstants.Shooter.toRotations(targetRadians);

		// Convert rad/s → mechanism rot/s — again NO manual gear ratio, Phoenix handles it
		double velocityMechRotPerSec = velocityRadPerSec / RobotConstants.Shooter.ROT_TO_RAD;

		turretMotorRequest
				.withPosition(targetRotations)
				.withVelocity(velocityMechRotPerSec); // Kraken interpolates at 1kHz using kV

		return turretMotor.setControl(turretMotorRequest);
	}

	@Override
	public void setTurretVoltage(double voltage) {
		turretMotorVoltageRequest.withOutput(voltage);
		turretMotor.setControl(turretMotorVoltageRequest);
	}

	@Override
	public double getTurretPosition() {
		return turretMotor.getPosition().getValueAsDouble() * RobotConstants.Shooter.ROT_TO_RAD;
	}

	@Override
	public double getTurretTargetPosition() {
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
		homingLimit.StatorCurrentLimit = 10.0;
		homingLimit.StatorCurrentLimitEnable = true;
		turretMotor.getConfigurator().apply(homingLimit);

		setTurretVoltage(slowVolts);

		double current = turretMotor.getStatorCurrent().getValueAsDouble();
		double velocity = Math.abs(turretMotor.getVelocity().getValueAsDouble());

		boolean isStalled = (current > 8.0) && (velocity < 1.0);

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

			double homeRotations = homeRadians / RobotConstants.Shooter.ROT_TO_RAD;
			turretMotor.setPosition(homeRotations);

			var normalLimit = new CurrentLimitsConfigs();
			normalLimit.StatorCurrentLimit = 30.0;
			normalLimit.StatorCurrentLimitEnable = true;
			turretMotor.getConfigurator().apply(normalLimit);
			homed = true;
		}
		return homed;
	}

	@Override
	public void onShootSimulation() {}
}
