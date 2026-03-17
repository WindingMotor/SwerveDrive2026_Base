// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.devices.IRBeamBreak;

public class IO_IntakeRealBoxBot implements IO_IntakeBase {

	private final TalonFX intakeMotor;
	private final VoltageOut intakeMotorRequest;

	private final IRBeamBreak sensor;

	public IO_IntakeRealBoxBot(TalonFXConfiguration intakeMotorConfiguration) {

		intakeMotor = new TalonFX(0, RobotConstants.CANBUS_CANIVORE);
		intakeMotor.getConfigurator().apply(intakeMotorConfiguration);
		intakeMotorRequest = new VoltageOut(0.0);

		sensor = new IRBeamBreak(1);
	}

	@Override
	public void updateInputs(IntakeInputs inputs) {
		inputs.intakeVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
		inputs.intakeTargetVoltage = intakeMotorRequest.getOutputMeasure().in(Volts);
		inputs.intakeCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();

		inputs.sliderPosition = 0.0;
		inputs.sliderTargetPosition = 0.0;
		inputs.sliderCurrent = 0.0;

		inputs.sensor = sensor.getValueAsBoolean();
	}

	@Override
	public StatusCode setIntakeVoltage(double voltage) {
		intakeMotorRequest.withOutput(voltage);
		return intakeMotor.setControl(intakeMotorRequest);
	}

	@Override
	public StatusCode setSliderVoltage(double voltage) {
		return StatusCode.kInvalidClass;
	}
}
