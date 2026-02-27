// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.devices.IRBeamBreak;

public class IO_IndexerReal implements IO_IndexerBase {

	private final TalonFX spinnerMotor;
	private final VoltageOut spinnerMotorRequest;

	private final TalonFX kickerMotor;
	private final VoltageOut kickerVoltageRequest;

	private final TalonFX climbMotor;
	private final VoltageOut climbVoltageRequest;

	private final IRBeamBreak sensor;

	public IO_IndexerReal(
			TalonFXConfiguration spinnerMotorConfiguration,
			TalonFXConfiguration kickerMotorConfiguration,
			TalonFXConfiguration climbMotorConfiguration) {

		spinnerMotor =
				new TalonFX(RobotConstants.Indexer.SPINNER_MOTOR_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		spinnerMotor.getConfigurator().apply(spinnerMotorConfiguration);
		spinnerMotorRequest = new VoltageOut(0.0);

		kickerMotor =
				new TalonFX(RobotConstants.Indexer.KICKER_MOTOR_CAN_ID, RobotConstants.CANBUS_CANIVORE);
		kickerMotor.getConfigurator().apply(kickerMotorConfiguration);
		kickerVoltageRequest = new VoltageOut(0.0);

		climbMotor = new TalonFX(33, RobotConstants.CANBUS_CANIVORE);
		climbMotor.getConfigurator().apply(climbMotorConfiguration);
		climbVoltageRequest = new VoltageOut(0.0);

		sensor = new IRBeamBreak(0);
	}

	@Override
	public void updateInputs(IndexerInputs inputs) {
		inputs.spinnerVoltage = spinnerMotor.getMotorVoltage().getValueAsDouble();
		inputs.spinnerTargetVoltage = spinnerMotorRequest.getOutputMeasure().in(Volts);
		inputs.spinnerCurrent = spinnerMotor.getStatorCurrent().getValueAsDouble();

		inputs.kickerVoltage = kickerMotor.getMotorVoltage().getValueAsDouble();
		inputs.kickerTargetVoltage = kickerVoltageRequest.getOutputMeasure().in(Volts);
		inputs.kickerCurrent = kickerMotor.getStatorCurrent().getValueAsDouble();

		inputs.climbVoltage = climbMotor.getMotorVoltage().getValueAsDouble();
		inputs.climbTargetVoltage = climbVoltageRequest.getOutputMeasure().in(Volts);
		inputs.climbCurrent = climbMotor.getStatorCurrent().getValueAsDouble();

		inputs.sensor = sensor.getValueAsBoolean();
	}

	@Override
	public StatusCode setSpinnerVoltage(double voltage) {
		spinnerMotorRequest.withOutput(voltage);
		return spinnerMotor.setControl(spinnerMotorRequest);
	}

	@Override
	public StatusCode setKickerVoltage(double voltage) {
		kickerVoltageRequest.withOutput(voltage);
		return kickerMotor.setControl(kickerVoltageRequest);
	}

	@Override
	public StatusCode setClimbVoltage(double voltage) {
		climbVoltageRequest.withOutput(voltage);
		return climbMotor.setControl(climbVoltageRequest);
	}
}
