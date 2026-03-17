// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import org.littletonrobotics.junction.Logger;

public class SUB_Shooter extends SubsystemBase {
	private final IO_ShooterBase io;
	private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	public SUB_Shooter(IO_ShooterBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);
	}

	public Pair<StatusCode, StatusCode> setShooterVelocities(double velocity) {
		return io.setShooterVelocities(velocity);
	}

	/**
	 * Sets turret position with velocity feedforward for 1kHz Kraken interpolation. Called at 100Hz
	 * from Robot.java's addPeriodic loop.
	 *
	 * @param position Target angle in radians
	 * @param velocityRadPerSec Setpoint velocity in rad/s (derivative of position setpoint)
	 */
	public StatusCode setTurretPosition(double position, double velocityRadPerSec) {
		return io.setTurretPosition(position, velocityRadPerSec);
	}

	/** Zero-velocity overload — used by homing and any static setpoint callers. */
	public StatusCode setTurretPosition(double position) {
		return io.setTurretPosition(position, 0.0);
	}

	public void setShooterVoltages(double voltages) {
		io.setShooterVoltages(voltages);
	}

	public double getTurretPosition() {
		return io.getTurretPosition();
	}

	public double getTurretTargetPosition() {
		return io.getTurretTargetPosition();
	}

	public double getShooterVelocityRPMSetpoint() {
		return inputs.shooterMotorOneTargetVelocity;
	}

	public boolean isTurretAtTarget() {
		double currentAngle = getTurretPosition();
		double targetRad = getTurretTargetPosition();
		return Math.abs(targetRad - currentAngle) < RobotConstants.Shooter.TURRET_ANGLE_OFFSET;
	}

	public boolean isShooterAtSpeed(double targetRPM, double toleranceRPM) {
		return Math.abs(inputs.shooterMotorOneVelocity - targetRPM) < toleranceRPM
				&& Math.abs(inputs.shooterMotorTwoVelocity - targetRPM) < toleranceRPM;
	}

	public boolean isShooterAtSpeed() {
		return isShooterAtSpeed(inputs.shooterMotorOneTargetVelocity, RobotConstants.Shooter.SHOOTER_RPM_TOLERANCE);
	}

	public boolean isReadyToShoot() {
		return isTurretAtTarget() && isShooterAtSpeed();
	}

	public double getAverageShooterVelocity() {
		return (inputs.shooterMotorOneVelocity + inputs.shooterMotorTwoVelocity) / 2.0;
	}

	public void setTurretVoltage(double voltage) {
		io.setTurretVoltage(voltage);
	}

	public Boolean homeTurret(Boolean homed) {
		return homed;
	}

	public void onShoot() {
		io.onShootSimulation();
	}
}
