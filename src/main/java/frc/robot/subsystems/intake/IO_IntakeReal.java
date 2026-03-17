// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.devices.IRBeamBreak;

public class IO_IntakeReal implements IO_IntakeBase {

	private final TalonFX intakeMotor;
	private final VoltageOut intakeMotorRequest;

	private final TalonFX sliderMotor;
	private final PositionVoltage sliderMotorRequest;

	private final IRBeamBreak sensor;

	public IO_IntakeReal(
			TalonFXConfiguration intakeMotorConfiguration,
			TalonFXConfiguration sliderMotorConfiguration) {

		intakeMotor =
				new TalonFX(RobotConstants.Intake.INTAKE_MOTOR_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		intakeMotor.getConfigurator().apply(intakeMotorConfiguration);
		intakeMotorRequest = new VoltageOut(0.0);

		sliderMotor =
				new TalonFX(RobotConstants.Intake.SLIDER_MOTOR_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		sliderMotor.getConfigurator().apply(sliderMotorConfiguration);
		sliderMotor.setPosition(0.0);
		sliderMotorRequest = new PositionVoltage(0.0);

		sensor = new IRBeamBreak(1);
	}

	@Override
	public void updateInputs(IntakeInputs inputs) {
		inputs.intakeVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
		inputs.intakeTargetVoltage = intakeMotorRequest.getOutputMeasure().in(Volts);
		inputs.intakeCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();

		inputs.sliderPosition = sliderMotor.getPosition().getValueAsDouble();
		inputs.sliderTargetPosition =
				sliderMotorRequest
						.Position; // TODO: Might need to multiply by motor conversion factor in future.
		inputs.sliderCurrent = sliderMotor.getStatorCurrent().getValueAsDouble();

		inputs.sensor = sensor.getValueAsBoolean();
	}

	@Override
	public StatusCode setIntakeVoltage(double voltage) {
		intakeMotorRequest.withOutput(voltage);
		return intakeMotor.setControl(intakeMotorRequest);
	}

	@Override
	public StatusCode setSliderVoltage(double voltage) {
		sliderMotorRequest.withPosition(voltage);
		return sliderMotor.setControl(sliderMotorRequest);
	}
}
