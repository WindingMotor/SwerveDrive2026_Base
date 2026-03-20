// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusCode;
import org.littletonrobotics.junction.AutoLog;

public interface IO_IntakeBase {

	@AutoLog
	public static class IntakeInputs {

		public double intakeVoltage = 0.0;
		public double intakeTargetVoltage = 0.0;
		public double intakeCurrent = 0.0;

		public double sliderPosition = 0.0;
		public double sliderTargetPosition = 0.0;
		public double sliderVoltage = 0.0;
		public double sliderTargetVoltage = 0.0;
		public double sliderCurrent = 0.0;

		public boolean sensor = false;
	}

	public void updateInputs(IntakeInputs inputs);

	public StatusCode setIntakeVoltage(double voltage);

	public StatusCode setSliderVoltage(double voltage);

	public StatusCode setSliderPosition(double meters);
}
