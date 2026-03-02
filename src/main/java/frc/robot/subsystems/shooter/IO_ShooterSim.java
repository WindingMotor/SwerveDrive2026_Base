// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.util.fieldsim.SimulationManager;

public class IO_ShooterSim implements IO_ShooterBase {

	// --- Hardware (runs in sim via CTRE SimState) ---
	private final TalonFX shooterMotorOne;
	private final TalonFX shooterMotorTwo;
	private final TalonFX turretMotor;

	// --- Control requests (identical to real impl) ---
	private final VelocityVoltage shooterMotorsRequest = new VelocityVoltage(0.0);
	private final VoltageOut shooterVoltageRequest = new VoltageOut(0.0);
	private final PositionVoltage turretPositionRequest = new PositionVoltage(0.0);
	private final VoltageOut turretVoltageRequest = new VoltageOut(0.0);

	// --- Physics models ---

	// Both shooter motors act on ONE shared flywheel mass.
	// DCMotor.getKrakenX60Foc(2) models two motors summing torque onto that single inertia.
	// 0.004 kg·m² is a reasonable starting estimate for a dual-wheel shooter flywheel —
	// increase if spin-up feels unrealistically fast compared to the real robot.
	private static final double SHOOTER_GEAR_RATIO = 1.0;
	private final DCMotorSim shooterSimModel =
			new DCMotorSim(
					LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.004, SHOOTER_GEAR_RATIO),
					DCMotor.getKrakenX60Foc(2));

	// Turret: single Kraken through a 154:11 gearbox, ~0.01 kg·m² mechanism inertia
	private static final double TURRET_GEAR_RATIO =
			154.0 / 11.0; // matches SensorToMechanismRatio in config
	private final DCMotorSim turretSimModel =
			new DCMotorSim(
					LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.01, TURRET_GEAR_RATIO),
					DCMotor.getKrakenX60Foc(1));

	// Independent position accumulator so soft-limit clamping works correctly
	private double simTurretPositionRad = 0.0;

	private boolean simHomed = false;

	public IO_ShooterSim(
			TalonFXConfiguration shooterMotorOneConfiguration,
			TalonFXConfiguration shooterMotorTwoConfiguration,
			TalonFXConfiguration turretMotorConfiguration) {

		// Note: sim devices don't need a CANivore bus name
		shooterMotorOne = new TalonFX(RobotConstants.Shooter.SHOOTER_MOTOR_ONE_CAN_ID);
		shooterMotorOne.getConfigurator().apply(shooterMotorOneConfiguration);

		shooterMotorTwo = new TalonFX(RobotConstants.Shooter.SHOOTER_MOTOR_TWO_CAN_ID);
		shooterMotorTwo.getConfigurator().apply(shooterMotorTwoConfiguration);

		turretMotor = new TalonFX(RobotConstants.Shooter.TURRET_MOTOR_CAN_ID);
		turretMotor.getConfigurator().apply(turretMotorConfiguration);

		// Mirror real robot invert settings so PID signs are consistent
		shooterMotorOne.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
		shooterMotorTwo.getSimState().Orientation = ChassisReference.Clockwise_Positive;
		turretMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
	}

	// -------------------------------------------------------------------------
	// Physics update — driven from updateInputs() once per 20 ms loop
	// -------------------------------------------------------------------------
	private void updateSimulation() {
		TalonFXSimState simOne = shooterMotorOne.getSimState();
		TalonFXSimState simTwo = shooterMotorTwo.getSimState();
		TalonFXSimState simTurret = turretMotor.getSimState();

		double batteryVoltage = RobotController.getBatteryVoltage();
		simOne.setSupplyVoltage(batteryVoltage);
		simTwo.setSupplyVoltage(batteryVoltage);
		simTurret.setSupplyVoltage(batteryVoltage);

		// --- Shared flywheel ---
		// Average the two motor output voltages as input to the shared mass model.
		// This correctly represents two motors driving one load: each contributes
		// half the current but together they produce the full combined torque.
		double motorVoltageOne = simOne.getMotorVoltageMeasure().in(Volts);
		double motorVoltageTwo = simTwo.getMotorVoltageMeasure().in(Volts);
		double sharedVoltage = (motorVoltageOne + motorVoltageTwo) / 2.0;

		shooterSimModel.setInputVoltage(sharedVoltage);
		shooterSimModel.update(0.020);

		// Write the shared result back to both motor SimStates
		var sharedPosition = shooterSimModel.getAngularPosition().times(SHOOTER_GEAR_RATIO);
		var sharedVelocity = shooterSimModel.getAngularVelocity().times(SHOOTER_GEAR_RATIO);

		simOne.setRawRotorPosition(sharedPosition);
		simOne.setRotorVelocity(sharedVelocity);
		simTwo.setRawRotorPosition(sharedPosition);
		simTwo.setRotorVelocity(sharedVelocity);

		// --- Turret ---
		double turretVoltage = simTurret.getMotorVoltageMeasure().in(Volts);
		turretSimModel.setInputVoltage(turretVoltage);
		turretSimModel.update(0.020);

		// Integrate mechanism velocity to get position in radians
		simTurretPositionRad += turretSimModel.getAngularVelocity().in(RadiansPerSecond) * 0.020;

		// Enforce soft limits in the physics model
		simTurretPositionRad =
				Math.max(
						RobotConstants.Shooter.TURRET_RADIANS_MIN,
						Math.min(RobotConstants.Shooter.TURRET_RADIANS_MAX, simTurretPositionRad));

		// Convert mechanism position (rad) → rotor position (rotations) for CTRE
		double rotorPositionRot =
				(simTurretPositionRad / RobotConstants.Shooter.ROT_TO_RAD) * TURRET_GEAR_RATIO;
		double rotorVelocityRps =
				(turretSimModel.getAngularVelocity().in(RadiansPerSecond)
						/ RobotConstants.Shooter.ROT_TO_RAD);

		simTurret.setRawRotorPosition(rotorPositionRot);
		simTurret.setRotorVelocity(rotorVelocityRps);
	}

	// -------------------------------------------------------------------------
	// IO_ShooterBase
	// -------------------------------------------------------------------------

	@Override
	public void updateInputs(ShooterInputs inputs) {
		updateSimulation();

		// Both motors read the same shared flywheel state; convert rps → RPM
		inputs.shooterMotorOneVelocity = shooterMotorOne.getVelocity().getValueAsDouble() * 60.0;
		inputs.shooterMotorOneTargetVelocity = shooterMotorsRequest.getVelocityMeasure().in(RPM);
		inputs.shooterMotorOneCurrent =
				shooterMotorOne.getSimState().getSupplyCurrentMeasure().in(Amps);

		inputs.shooterMotorTwoVelocity = shooterMotorTwo.getVelocity().getValueAsDouble() * 60.0;
		inputs.shooterMotorTwoTargetVelocity = shooterMotorsRequest.getVelocityMeasure().in(RPM);
		inputs.shooterMotorTwoCurrent =
				shooterMotorTwo.getSimState().getSupplyCurrentMeasure().in(Amps);

		inputs.turretMotorCurrentPosition =
				turretMotor.getPosition().getValueAsDouble() * RobotConstants.Shooter.ROT_TO_RAD;
		inputs.turretMotorCurrentTargetPosition =
				turretPositionRequest.Position * RobotConstants.Shooter.ROT_TO_RAD;
		inputs.turretMotorCurrent = turretMotor.getSimState().getSupplyCurrentMeasure().in(Amps);

		// Homing sensor fires when turret is within 0.05 rad of its max limit
		inputs.turretPositionSensor =
				simTurretPositionRad >= RobotConstants.Shooter.TURRET_RADIANS_MAX - 0.05;
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

	@Override
	public void setShooterVoltages(double voltages) {
		shooterVoltageRequest.withOutput(voltages);
		shooterMotorOne.setControl(shooterVoltageRequest);
		shooterMotorTwo.setControl(shooterVoltageRequest);
	}

	@Override
	public StatusCode setTurretPosition(double radians) {
		double clamped =
				Math.max(
						RobotConstants.Shooter.TURRET_RADIANS_MIN,
						Math.min(RobotConstants.Shooter.TURRET_RADIANS_MAX, radians));
		turretPositionRequest.withPosition(RobotConstants.Shooter.toRotations(clamped));
		return turretMotor.setControl(turretPositionRequest);
	}

	@Override
	public void setTurretVoltage(double voltage) {
		turretVoltageRequest.withOutput(voltage);
		turretMotor.setControl(turretVoltageRequest);
	}

	@Override
	public double getTurretPosition() {
		return turretMotor.getPosition().getValueAsDouble() * RobotConstants.Shooter.ROT_TO_RAD;
	}

	@Override
	public double getTurretTargetPosition() {
		return turretMotor.getPosition().getValueAsDouble() * RobotConstants.Shooter.ROT_TO_RAD;
	}

	@Override
	public Boolean homeTurret(Boolean homed) {
		if (!simHomed) {
			setTurretVoltage(RobotConstants.Shooter.TURRET_SLOW_MOVE_VOLTAGE);
			if (simTurretPositionRad >= RobotConstants.Shooter.TURRET_RADIANS_MAX - 0.05) {
				turretMotor.setPosition(
						RobotConstants.Shooter.toRotations(RobotConstants.Shooter.TURRET_RADIANS_MAX));
				simTurretPositionRad = RobotConstants.Shooter.TURRET_RADIANS_MAX;
				simHomed = true;
			}
		}
		return simHomed;
	}

	// For simulated shoot
	@Override
	public void onShootSimulation() {
		if (RobotConstants.ENABLE_SIM_MANAGER) {
			double wheelCircumference = Math.PI * RobotConstants.Shooter.SHOOTER_WHEEL_DIAMETER_METERS;
			double wheelSurfaceSpeed =
					(shooterSimModel.getAngularVelocityRPM() / 60.0) * wheelCircumference;
			double exitVelocity = wheelSurfaceSpeed * RobotConstants.Shooter.SHOOTER_EFFICIENCY_FACTOR;

			SimulationManager.getInstance().launchFuel(exitVelocity, simTurretPositionRad);
		}
	}
}
