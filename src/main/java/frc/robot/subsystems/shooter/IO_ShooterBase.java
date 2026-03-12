// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.Pair;
import org.littletonrobotics.junction.AutoLog;

public interface IO_ShooterBase {

	@AutoLog
	public static class ShooterInputs {

		public double shooterMotorOneVelocity = 0.0;
		public double shooterMotorOneTargetVelocity = 0.0;
		public double shooterMotorOneCurrent = 0.0;

		public double shooterMotorTwoVelocity = 0.0;
		public double shooterMotorTwoTargetVelocity = 0.0;
		public double shooterMotorTwoCurrent = 0.0;

		public double turretMotorCurrentPosition = 0.0;
		public double turretMotorCurrentTargetPosition = 0.0;
		public double turretMotorCurrent = 0.0;

		public boolean turretPositionSensor = false;
		public double turretVelocity = 0.0;
	}

	public default void updateInputs(ShooterInputs inputs) {}

	public Pair<StatusCode, StatusCode> setShooterVelocities(double velocity);

	public void setShooterVoltages(double voltages);

	/**
	 * Sets turret position with a velocity feedforward hint. The velocity (mechanism rot/s) is passed
	 * to the Kraken via .withVelocity() so its 1kHz onboard PID can interpolate between 10ms Java
	 * updates using kV.
	 *
	 * @param radians Target position in radians
	 * @param velocityRadPerSec How fast the setpoint is moving (rad/s) — used for kV feedforward
	 */
	public StatusCode setTurretPosition(double radians, double velocityRadPerSec);

	/** Zero-velocity overload — used by homing, sim, and any caller that doesn't track velocity. */
	public default StatusCode setTurretPosition(double radians) {
		return setTurretPosition(radians, 0.0);
	}

	public void setTurretVoltage(double voltage);

	public double getTurretPosition();

	public double getTurretTargetPosition();

	public Boolean homeTurret(Boolean homed);

	public default void onShootSimulation() {}
}
