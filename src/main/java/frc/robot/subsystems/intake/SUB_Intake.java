// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_Intake extends SubsystemBase {

	private final IO_IntakeBase io;

	private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	public SUB_Intake(IO_IntakeBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {

		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);
	}

	public StatusCode setIntakeVoltage(double voltage) {
		return io.setIntakeVoltage(voltage);
	}

	public StatusCode setSliderVoltage(double voltage) {
		return io.setSliderVoltage(voltage);
	}

	public StatusCode setSliderPosition(double meters) {
		return io.setSliderPosition(meters);
	}

	public boolean isIntakeRunning() {
		return inputs.intakeTargetVoltage != 0.0;
	}

	public boolean getIntakeSensor() {
		return io.getIntakeSensor();
	}

	public void setNewSliderPosition(double meters) {
		io.setNewSliderPosition(meters);
	}
}
