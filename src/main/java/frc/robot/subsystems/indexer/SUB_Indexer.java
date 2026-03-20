// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_Indexer extends SubsystemBase {

	private final IO_IndexerBase io;
	private final IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();

	public SUB_Indexer(IO_IndexerBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Indexer", inputs);
	}

	public StatusCode setSpinnerVoltage(double voltage) {
		return io.setSpinnerVoltage(voltage);
	}

	public StatusCode setKickerVoltage(double voltage) {
		return io.setKickerVoltage(voltage);
	}

	public StatusCode setClimbVoltage(double voltage) {
		return io.setClimbVoltage(voltage);
	}

	public StatusCode setClimbPosition(double meters) {
		return io.setClimbPosition(meters);
	}
}
