// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.generic.CMD_Superstructure;
import frc.robot.constants.LIB_DriveConstants;
import frc.robot.constants.LIB_VisionConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotMode;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.lib.windingmotor.drive.gyro.*;
import frc.robot.lib.windingmotor.drive.module.*;
import frc.robot.lib.windingmotor.util.fieldsim.SimulationManager;
import frc.robot.lib.windingmotor.vision.IO_VisionCamera;
import frc.robot.lib.windingmotor.vision.SUB_Vision;
import frc.robot.subsystems.indexer.IO_IndexerReal;
import frc.robot.subsystems.indexer.SUB_Indexer;
import frc.robot.subsystems.intake.IO_IntakeRealNew;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.shooter.IO_ShooterReal;
import frc.robot.subsystems.shooter.IO_ShooterSim;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.superstructure.SUB_Superstructure;
import frc.robot.subsystems.superstructure.SUB_Superstructure.RobotState;

@SuppressWarnings("unused")
public class RobotContainer {

	// ====================================================================
	// Auto
	// ====================================================================

	public static final String AUTO_NAME =
			"RIGHT_T2_HP_NEW"; // middle = MIDDLE_HP and RIGHT_T2_HP_NEW   /   right = RIGHT_T2_HP   /
	// left = LEFT_T2_Comp
	public static Command AUTO_COMMAND;

	// ====================================================================
	// Controllers
	// ====================================================================

	private CommandXboxController driverController;
	private CommandXboxController operatorController;
	private CommandXboxController turretTestcontroller;

	// ====================================================================
	// Subsystems
	// ====================================================================

	private SUB_Indexer indexer;
	private SUB_Intake intake;
	private SUB_Shooter shooter;

	private Drive drive;
	private SUB_Vision vision;

	private SUB_Superstructure superstructure;

	// ====================================================================
	// Constructor
	// ====================================================================

	public RobotContainer() {
		initializeControllers();
		initializeSubsystems();
		configurePathplannerCommands();
		configureDefaultCommands();
		configureButtonBindings();

		AUTO_COMMAND = AutoBuilder.buildAuto(AUTO_NAME);

		if (RobotConstants.ROBOT_MODE == RobotMode.SIM && RobotConstants.ENABLE_SIM_MANAGER) {
			configureSim();
		}
	}

	// ====================================================================
	// Initialization
	// ====================================================================

	private void initializeControllers() {
		driverController = new CommandXboxController(0);
		operatorController = new CommandXboxController(1);
		turretTestcontroller = new CommandXboxController(2);
	}

	private void initializeSubsystems() {
		indexer =
				new SUB_Indexer(
						new IO_IndexerReal(
								RobotConstants.Indexer.KICKER_MOTOR_CONFIG,
								RobotConstants.Indexer.SPINNER_MOTOR_CONFIG,
								RobotConstants.Indexer.CLIMB_MOTOR_CONFIG));

		intake =
				new SUB_Intake(
						new IO_IntakeRealNew(
								RobotConstants.Intake.INTAKE_MOTOR_CONFIG,
								RobotConstants.Intake.SLIDER_MOTOR_CONFIG));

		switch (RobotConstants.ROBOT_MODE) {
			case REAL:
				drive =
						new Drive(
								new IO_GyroReal(),
								new IO_ModuleReal(LIB_DriveConstants.FrontLeft),
								new IO_ModuleReal(LIB_DriveConstants.FrontRight),
								new IO_ModuleReal(LIB_DriveConstants.BackLeft),
								new IO_ModuleReal(LIB_DriveConstants.BackRight));

				shooter =
						new SUB_Shooter(
								new IO_ShooterReal(
										RobotConstants.Shooter.SHOOTER_MOTOR_ONE_CONFIG,
										RobotConstants.Shooter.SHOOTER_MOTOR_TWO_CONFIG,
										RobotConstants.Shooter.TURRET_MOTOR_CONFIG));
				break;

			case SIM:
				drive =
						new Drive(
								new IO_GyroBase() {},
								new IO_ModuleSim(LIB_DriveConstants.FrontLeft),
								new IO_ModuleSim(LIB_DriveConstants.FrontRight),
								new IO_ModuleSim(LIB_DriveConstants.BackLeft),
								new IO_ModuleSim(LIB_DriveConstants.BackRight));

				shooter =
						new SUB_Shooter(
								new IO_ShooterSim(
										RobotConstants.Shooter.SHOOTER_MOTOR_ONE_CONFIG,
										RobotConstants.Shooter.SHOOTER_MOTOR_TWO_CONFIG,
										RobotConstants.Shooter.TURRET_MOTOR_CONFIG));
				break;

			default:
				drive =
						new Drive(
								new IO_GyroBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {},
								new IO_ModuleBase() {});
				break;
		}

		vision =
				new SUB_Vision(
						drive::addVisionMeasurement,
						new IO_VisionCamera(
								LIB_VisionConstants.camera0Name, LIB_VisionConstants.robotToCamera0),
						new IO_VisionCamera(
								LIB_VisionConstants.camera1Name, LIB_VisionConstants.robotToCamera1));
		/*new IO_VisionCamera(
		LIB_VisionConstants.camera1Name, LIB_VisionConstants.robotToCamera2));*/
		// TODO: Where is other camera ben?

		superstructure = new SUB_Superstructure(indexer, intake, shooter, drive);
	}

	// ====================================================================
	// Configuration
	// ====================================================================

	private void configureDefaultCommands() {
		drive.setDefaultCommand(
				DriveCommands.driveNormalExpo(
						drive,
						() -> -driverController.getRawAxis(1),
						() -> -driverController.getRawAxis(0),
						() -> -driverController.getRawAxis(4),
						1.0, // A VALUE OF 1.0 is FULL ROBOT SPEED
						0.8, // Keep rotation conservative
						0.1, // Movement expo
						0.2)); // Rotation expo
	}

	private void configureButtonBindings() {

		// --- Intake ---
		driverController
				.leftTrigger()
				.onTrue(new CMD_Superstructure(superstructure, RobotState.INTAKE_IN));
		driverController
				.leftTrigger()
				.onFalse(new CMD_Superstructure(superstructure, RobotState.INTAKE));

		operatorController
				.leftTrigger()
				.onTrue(new CMD_Superstructure(superstructure, RobotState.INTAKE_IN));
		operatorController
				.leftTrigger()
				.onFalse(new CMD_Superstructure(superstructure, RobotState.INTAKE));

		operatorController
				.rightTrigger()
				.onTrue(new CMD_Superstructure(superstructure, RobotState.EJECT));
		operatorController
				.rightTrigger()
				.onFalse(new CMD_Superstructure(superstructure, RobotState.INTAKE));

		// --- Shoot ---
		driverController
				.rightTrigger()
				.onTrue(new CMD_Superstructure(superstructure, RobotState.SHOOTING));
		driverController
				.rightTrigger()
				.onFalse(new CMD_Superstructure(superstructure, RobotState.READY));

		operatorController.x().onTrue(new CMD_Superstructure(superstructure, RobotState.SHOOTING));
		operatorController.x().onFalse(new CMD_Superstructure(superstructure, RobotState.READY));

		operatorController.b().onTrue(new CMD_Superstructure(superstructure, RobotState.IDLE));

		// --- Climb ---
		driverController.povUp().onTrue(new CMD_Superstructure(superstructure, RobotState.CLIMB_UP));
		driverController.povUp().onFalse(new CMD_Superstructure(superstructure, RobotState.CLIMB_STOP));

		driverController
				.povDown()
				.onTrue(new CMD_Superstructure(superstructure, RobotState.CLIMB_DOWN));
		driverController
				.povDown()
				.onFalse(new CMD_Superstructure(superstructure, RobotState.CLIMB_STOP));
	}

	private void configurePathplannerCommands() {
		NamedCommands.registerCommand("IDLE", new CMD_Superstructure(superstructure, RobotState.IDLE));
		NamedCommands.registerCommand(
				"SHOOT", new CMD_Superstructure(superstructure, RobotState.SHOOTING));
		NamedCommands.registerCommand(
				"INTAKE", new CMD_Superstructure(superstructure, RobotState.INTAKE_AUTO));
		NamedCommands.registerCommand(
				"INTAKE_IN", new CMD_Superstructure(superstructure, RobotState.INTAKE_IN));
		NamedCommands.registerCommand(
				"READY", new CMD_Superstructure(superstructure, RobotState.READY));
		NamedCommands.registerCommand("CLIMB", new CMD_Superstructure(superstructure, RobotState.IDLE));
	}

	private void configureSim() {
		SimulationManager.getInstance().registerRobot(drive, intake, shooter);

		SmartDashboard.putData(
				"Reset Fuel",
				Commands.runOnce(() -> SimulationManager.getInstance().resetFuel()).ignoringDisable(true));
	}

	// ====================================================================
	// Autonomous
	// ====================================================================

	/**
	 * Returns the command to run in autonomous mode. Uses the cached PathPlanner auto command built
	 * at construction time.
	 */
	public Command getAutonomousCommand() {
		if (AUTO_COMMAND != null) {
			return AUTO_COMMAND;
		} else {
			return new PrintCommand("Auto Command is NULL!");
		}
	}

	// ====================================================================
	// Superstructure accessor for Robot.java addPeriodic
	// ====================================================================

	public SUB_Superstructure getSuperstructure() {
		return superstructure;
	}
}
