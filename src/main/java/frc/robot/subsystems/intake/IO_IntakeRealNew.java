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
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.RobotConstants;

public class IO_IntakeRealNew implements IO_IntakeBase {

	private final TalonFX intakeMotor;
	private final VoltageOut intakeMotorRequest;

	private final TalonFX sliderMotor;
	private final VoltageOut sliderMotorRequest;
	private final PositionVoltage sliderPositionRequest;

	private final DigitalInput intakeSensor = new DigitalInput(0);

	public IO_IntakeRealNew(
			TalonFXConfiguration intakeMotorConfiguration,
			TalonFXConfiguration sliderMotorConfiguration) {

		intakeMotor =
				new TalonFX(RobotConstants.Intake.INTAKE_MOTOR_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		intakeMotor.getConfigurator().apply(intakeMotorConfiguration);
		intakeMotorRequest = new VoltageOut(0.0);

		sliderMotor =
				new TalonFX(RobotConstants.Intake.SLIDER_MOTOR_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		sliderMotor.getConfigurator().apply(sliderMotorConfiguration);
		// sliderMotor.setPosition(0.0);
		sliderMotorRequest = new VoltageOut(0.0);
		sliderPositionRequest = new PositionVoltage(0.0);
	}

	@Override
	public void updateInputs(IntakeInputs inputs) {
		inputs.intakeVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
		inputs.intakeTargetVoltage = intakeMotorRequest.getOutputMeasure().in(Volts);
		inputs.intakeCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();

		inputs.sliderPosition = sliderMotor.getPosition().getValueAsDouble();
		inputs.sliderTargetPosition = sliderPositionRequest.Position;

		inputs.sliderVoltage = sliderMotor.getMotorVoltage().getValueAsDouble();
		inputs.sliderTargetVoltage =
				sliderMotorRequest
						.getOutputMeasure()
						.in(Volts); // TODO: Might need to multiply by motor conversion factor in future.

		inputs.sliderCurrent = sliderMotor.getStatorCurrent().getValueAsDouble();

		inputs.intakeIsOut = intakeSensor.get();
	}

	@Override
	public StatusCode setIntakeVoltage(double voltage) {
		intakeMotorRequest.withOutput(voltage);
		return intakeMotor.setControl(intakeMotorRequest);
	}

	@Override
	public StatusCode setSliderVoltage(double voltage) {

		sliderMotorRequest.withOutput(voltage);
		return sliderMotor.setControl(sliderMotorRequest);
	}

	@Override
	public StatusCode setSliderPosition(double meters) {
		sliderPositionRequest.withPosition(meters);
		return sliderMotor.setControl(sliderPositionRequest);
	}

	@Override
	public boolean getIntakeSensor() {
		return intakeSensor.get();
	}

	@Override
	public void setNewSliderPosition(double meters) {
		sliderMotor.setPosition(meters);
	}
}
