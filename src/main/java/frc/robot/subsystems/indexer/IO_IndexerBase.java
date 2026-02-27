// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;

public interface IO_IndexerBase {

	@AutoLog
	public static class IndexerInputs {

		public double spinnerVoltage = 0.0;
		public double spinnerTargetVoltage = 0.0;
		public double spinnerCurrent = 0.0;

		public double kickerVoltage = 0.0;
		public double kickerTargetVoltage = 0.0;
		public double kickerCurrent = 0.0;

		public double climbVoltage = 0.0;
		public double climbTargetVoltage = 0.0;
		public double climbCurrent = 0.0;

		public boolean sensor = false;
	}

	public void updateInputs(IndexerInputs inputs);

	public StatusCode setSpinnerVoltage(double voltage);

	public StatusCode setKickerVoltage(double voltage);

	public StatusCode setClimbVoltage(double voltage);
}
