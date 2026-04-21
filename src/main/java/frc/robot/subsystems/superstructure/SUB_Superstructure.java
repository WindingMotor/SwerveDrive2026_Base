// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.subsystems.indexer.SUB_Indexer;
import frc.robot.subsystems.intake.SUB_Intake;
import frc.robot.subsystems.shooter.SUB_Shooter;
import org.littletonrobotics.junction.Logger;

public class SUB_Superstructure extends SubsystemBase {

	// ====================================================================
	// Enums
	// ====================================================================

	public enum TurretTarget {
		BLUE_HUB(new Translation2d(4.62, 4.03)),
		RED_HUB(new Translation2d(11.91, 4.03)),

		BLUE_AIMING_TOP_CORNER(new Translation2d(4.0, 6.0)),
		RED_AIMING_TOP_CORNER(new Translation2d(12.5, 6.0)),
		BLUE_AIMING_BOTTOM_CORNER(new Translation2d(4.0, 2.0)),
		RED_AIMING_BOTTOM_CORNER(new Translation2d(12.5, 2.0)),

		RED_FAR_TOP_CORNER(new Translation2d(6.0, 6.0)),
		BLUE_FAR_TOP_CORNER(new Translation2d(10.5, 6.0)),
		RED_FAR_BOTTOM_CORNER(new Translation2d(6.0, 2.0)),
		BLUE_FAR_BOTTOM_CORNER(new Translation2d(10.5, 2.0));

		private final Translation2d position;

		TurretTarget(Translation2d position) {
			this.position = position;
		}

		public Translation2d getPosition() {
			return position;
		}
	}

	public enum RobotState {
		IDLE,
		SHOOTING,
		READY,
		UNJAM,
		INTAKE,
		INTAKE_SKIPPER,
		INTAKE_AUTO,
		INTAKE_IN,
		INTAKE_IN_OUT,
		INTAKE_LIMP,
		CLIMB_TOP,
		CLIMB_BOTTOM,
		CLIMB_UP,
		CLIMB_DOWN,
		CLIMB_STOP,
		EJECT,
		TEST1,
		TEST2,
		TEST3,
		TEST4
	}

	// ====================================================================
	// Subsystem References
	// ====================================================================

	private SUB_Indexer indexerRef;
	private SUB_Intake intakeRef;
	private SUB_Shooter shooterRef;
	private Drive driveRef;
	private CommandXboxController driverController;

	// ====================================================================
	// State
	// ====================================================================

	private RobotState currentRobotState = RobotState.IDLE;
	private Translation2d turretTargetPose = new Translation2d(4.631, 4.031);

	private Boolean activelyShooting = false;
	private Boolean activelyReady = false;

	private boolean isRedGlobal =
			DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

	// Tracks the previous turret angle setpoint so we can compute d(setpoint)/dt
	// for velocity feedforward. Updated every 10ms by the addPeriodic loop.
	private double previousTurretAngleRad = 0.0;

	private int intakeCounter = 0;

	private int stallCounter = 0;
	private int reverseCounter = 0;
	private boolean isReversing = false;
	private boolean isRunning = false;

	private static final double STALL_CURRENT_THRESHOLD = 18.0;

	private static final int STALL_CYCLE_COUNT = 25;

	private static final int REVERSE_CYCLES = 50;

	// ====================================================================
	// Telemetry cache — written at 100Hz by the Notifier methods,
	// read at 50Hz by periodic() for safe AdvantageKit logging.
	// ====================================================================

	// updateTurretAngle()
	private Pose2d cache_turretPositionPose = new Pose2d();
	private Pose2d cache_actualGoalPose = new Pose2d();
	private Pose2d cache_virtualGoalPose = new Pose2d();
	private Pose2d cache_actualAimPose = new Pose2d();
	private Pose2d cache_targetAimPose = new Pose2d();
	private double cache_rangeCenter = 0.0;
	private double cache_rawTurretTarget = 0.0;
	private double cache_targetAngleRobotRel = 0.0;
	private double cache_targetAngleFieldRel = 0.0;
	private double cache_currentAngle = 0.0;
	private double cache_turretDeltaDegrees = 0.0;
	private double cache_setpointVelocity = 0.0;

	// updateShooterVelocity()
	private double cache_shooterDistance = 0.0;
	private double cache_shooterTargetRPM = 0.0;
	private double cache_shooterAverageRPM = 0.0;
	private Pose2d[] cache_shotLine = new Pose2d[] {new Pose2d(), new Pose2d()};

	// calculateVirtualGoal() — called from both 100Hz methods, cache latest values
	private Pose2d cache_motionComp_actualGoal = new Pose2d();
	private Pose2d cache_motionComp_virtualGoal = new Pose2d();
	private double cache_motionComp_distToActual = 0.0;
	private double cache_motionComp_targetRPM = 0.0;
	private double cache_motionComp_exitVelocity = 0.0;
	private double cache_motionComp_horizVelocity = 0.0;
	private double cache_motionComp_flightTime = 0.0;
	private double cache_motionComp_robotVelX = 0.0;
	private double cache_motionComp_robotVelY = 0.0;
	private double cache_motionComp_displacementX = 0.0;
	private double cache_motionComp_displacementY = 0.0;
	private double cache_motionComp_compOffset = 0.0;

	// ====================================================================
	// Constants
	// ====================================================================

	private final double INTAKE_MAX_EXTENSION_METERS = 12.7;
	private final double INTAKE_SLIDER_VOLTS = 5.0;

	// dt for the 100Hz addPeriodic loop — used to differentiate the setpoint
	private static final double TURRET_UPDATE_DT = 0.010;

	private final InterpolatingDoubleTreeMap shooterRPMTable;

	private final Translation2d TURRET_OFFSET_ROBOT =
			new Translation2d(
					RobotConstants.Shooter.TURRET_OFFSET_X_METERS,
					RobotConstants.Shooter.TURRET_OFFSET_Y_METERS);

	// ====================================================================
	// Other
	// ====================================================================

	private Timekeeper timekeeper;

	// ====================================================================
	// Constructor
	// ====================================================================

	public SUB_Superstructure(
			SUB_Indexer indexerRef,
			SUB_Intake intakeRef,
			SUB_Shooter shooterRef,
			Drive driveRef,
			CommandXboxController driverController) {
		this.indexerRef = indexerRef;
		this.intakeRef = intakeRef;
		this.shooterRef = shooterRef;
		this.driveRef = driveRef;
		this.driverController = driverController;

		shooterRPMTable = new InterpolatingDoubleTreeMap();
		for (double[] dataPoint : RobotConstants.Shooter.SHOOTER_RPM_DATA) {
			shooterRPMTable.put(dataPoint[0], dataPoint[1]);
		}

		timekeeper = new Timekeeper();
	}

	// ====================================================================
	// Periodic — 50Hz main loop
	// Only the state machine and ALL logging live here.
	// Turret/shooter updates run at 100Hz via addPeriodic in Robot.java.
	// ====================================================================

	@Override
	public void periodic() {
		// General state
		Logger.recordOutput("Superstructure/RobotState", currentRobotState.toString());
		Logger.recordOutput("Superstructure/ActivelyShooting", activelyShooting);
		Logger.recordOutput("Superstructure/ActivelyReady", activelyReady);
		Logger.recordOutput("Superstructure/RobotPose", driveRef.getPose());

		// Turret — cached by updateTurretAngle()
		Logger.recordOutput("Superstructure/Turret/TurretPosition", cache_turretPositionPose);
		Logger.recordOutput("Superstructure/Turret/ActualGoal", cache_actualGoalPose);
		Logger.recordOutput("Superstructure/Turret/VirtualGoal", cache_virtualGoalPose);
		Logger.recordOutput("Superstructure/Turret/ActualAim", cache_actualAimPose);
		Logger.recordOutput("Superstructure/Turret/TargetAim", cache_targetAimPose);
		Logger.recordOutput("Superstructure/Turret/RangeCenter", cache_rangeCenter);
		Logger.recordOutput("Superstructure/Turret/RawTurretTarget", cache_rawTurretTarget);
		Logger.recordOutput(
				"Superstructure/Turret/TargetAngleRobotRelative", cache_targetAngleRobotRel);
		Logger.recordOutput(
				"Superstructure/Turret/TargetAngleFieldRelative", cache_targetAngleFieldRel);
		Logger.recordOutput("Superstructure/Turret/CurrentAngle", cache_currentAngle);
		Logger.recordOutput("Superstructure/Turret/turretDeltaDegrees", cache_turretDeltaDegrees);
		Logger.recordOutput("Superstructure/Turret/SetpointVelocityRadPerSec", cache_setpointVelocity);
		Logger.recordOutput("Superstructure/Turret/isTurretAtTarget", shooterRef.isReadyToShoot());

		// Shooter — cached by updateShooterVelocity()
		Logger.recordOutput("Superstructure/Shooter/DistanceToVirtualGoal", cache_shooterDistance);
		Logger.recordOutput("Superstructure/Shooter/TargetRPM", cache_shooterTargetRPM);
		Logger.recordOutput("Superstructure/Shooter/AverageRPM", cache_shooterAverageRPM);
		Logger.recordOutput("Superstructure/Shooter/DistanceToVirtualGoalLine", cache_shotLine);

		// Motion compensation — cached by calculateVirtualGoal()
		Logger.recordOutput("Superstructure/MotionComp/ActualGoal", cache_motionComp_actualGoal);
		Logger.recordOutput("Superstructure/MotionComp/VirtualGoal", cache_motionComp_virtualGoal);
		Logger.recordOutput(
				"Superstructure/MotionComp/DistanceToActualGoal", cache_motionComp_distToActual);
		Logger.recordOutput("Superstructure/MotionComp/TargetRPM", cache_motionComp_targetRPM);
		Logger.recordOutput("Superstructure/MotionComp/ExitVelocityMPS", cache_motionComp_exitVelocity);
		Logger.recordOutput(
				"Superstructure/MotionComp/HorizontalVelocityMPS", cache_motionComp_horizVelocity);
		Logger.recordOutput("Superstructure/MotionComp/FlightTimeSeconds", cache_motionComp_flightTime);
		Logger.recordOutput("Superstructure/MotionComp/RobotVelocityX", cache_motionComp_robotVelX);
		Logger.recordOutput("Superstructure/MotionComp/RobotVelocityY", cache_motionComp_robotVelY);
		Logger.recordOutput("Superstructure/MotionComp/DisplacementX", cache_motionComp_displacementX);
		Logger.recordOutput("Superstructure/MotionComp/DisplacementY", cache_motionComp_displacementY);
		Logger.recordOutput(
				"Superstructure/MotionComp/CompensationOffsetMeters", cache_motionComp_compOffset);

		runStateMachine();

		timekeeper.update();
	}

	// ====================================================================
	// State Machine
	// ====================================================================

	private void runStateMachine() {
		switch (currentRobotState) {
			case IDLE:
				indexerRef.setSpinnerVoltage(0.0);
				indexerRef.setKickerVoltage(0.0);
				intakeRef.setIntakeVoltage(0.0);
				intakeRef.setSliderVoltage(0.0);
				indexerRef.setClimbVoltage(0.0);
				isRunning = false;
				break;

			case SHOOTING:
				isRunning = true;
				indexerRef.setKickerVoltage(10.0);
				break;

			case READY:
				isRunning = false;
				indexerRef.setKickerVoltage(0.0);
				break;

			case UNJAM:
				indexerRef.setSpinnerVoltage(-4.0);
				indexerRef.setKickerVoltage(-4.0);
				break;

			case INTAKE:
				intakeRef.setIntakeVoltage(10.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				break;

			case INTAKE_AUTO:
				intakeRef.setIntakeVoltage(9.5);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				break;

			case INTAKE_LIMP:
				intakeRef.setIntakeVoltage(4.0);
				intakeRef.setSliderVoltage(0.0);
				break;

			case EJECT:
				intakeRef.setIntakeVoltage(-10.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				break;

			case INTAKE_IN:
				intakeRef.setSliderPosition(0.0);
				intakeRef.setIntakeVoltage(2.0);
				break;

			case INTAKE_IN_OUT:
				if (intakeCounter < 30) {
					intakeRef.setSliderPosition(0.0);
					intakeCounter++;
				} else if (intakeCounter < 65) {
					intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
					intakeCounter++;
				} else {
					intakeCounter = 0;
				}
				intakeRef.setIntakeVoltage(4.0);
				break;

			case INTAKE_SKIPPER:
				intakeRef.setSliderVoltage(2.5); // (14.5);
				intakeRef.setIntakeVoltage(10.0);
				break;

			case CLIMB_TOP:
				indexerRef.setClimbPosition(100.0);
				break;

			case CLIMB_BOTTOM:
				indexerRef.setClimbPosition(1.0);
				break;

			case CLIMB_UP:
				indexerRef.setClimbVoltage(8.0);
				break;

			case CLIMB_DOWN:
				indexerRef.setClimbVoltage(-8.0);
				break;

			case CLIMB_STOP:
				indexerRef.setClimbVoltage(0.0);
				break;
		}
		runSpinner();
	}

	// ====================================================================
	// Turret & Shooter Updates — PUBLIC, called at 100Hz by Robot.java
	// NO Logger calls permitted here — use the cache fields above.
	// ====================================================================

	public void runSpinner() {

		if (!isRunning) {
			indexerRef.setSpinnerVoltage(0.0);
			stallCounter = 0;
			reverseCounter = 0;
			isReversing = false;
			return;
		}

		if (isReversing) {
			indexerRef.setSpinnerVoltage(-4.0);
			reverseCounter++;
			if (reverseCounter >= REVERSE_CYCLES) {
				isReversing = false;
				reverseCounter = 0;
				stallCounter = 0;
			}
		} else {
			indexerRef.setSpinnerVoltage(12.0);
		}

		if (indexerRef.getSpinnerCurrent() > STALL_CURRENT_THRESHOLD) {
			stallCounter++;
		} else {
			stallCounter = 0;
		}

		if (stallCounter >= STALL_CYCLE_COUNT) {
			isReversing = true;
			reverseCounter = 0;
			stallCounter = 0;
		}
	}

	/**
	 * Selects the correct turret target based on alliance and robot pose. Called at 100Hz via
	 * addPeriodic in Robot.java.
	 */
	public void updateTurretTarget() {
		isRedGlobal =
				DriverStation.getAlliance().isPresent()
						&& DriverStation.getAlliance().get() == Alliance.Red;

		Pose2d robotPose = driveRef.getPose();

		if (isRedGlobal) {
			if (robotPose.getX() > TurretTarget.RED_HUB.getPosition().getX()) { // if in red zone
				turretTargetPose = TurretTarget.RED_HUB.getPosition();

			} else if (robotPose.getX()
					> TurretTarget.BLUE_HUB.getPosition().getX()) { // if in middle zone
				if (robotPose.getY() > TurretTarget.RED_HUB.getPosition().getY()) {
					turretTargetPose = TurretTarget.RED_AIMING_TOP_CORNER.getPosition();
				} else {
					turretTargetPose = TurretTarget.RED_AIMING_BOTTOM_CORNER.getPosition();
				}

			} else { // if in blue zone
				if (robotPose.getY() > TurretTarget.RED_HUB.getPosition().getY()) {
					turretTargetPose = TurretTarget.RED_FAR_TOP_CORNER.getPosition();
				} else {
					turretTargetPose = TurretTarget.RED_FAR_BOTTOM_CORNER.getPosition();
				}
			}

		} else {
			if (robotPose.getX() < TurretTarget.BLUE_HUB.getPosition().getX()) {
				turretTargetPose = TurretTarget.BLUE_HUB.getPosition();

			} else if (robotPose.getX()
					< TurretTarget.RED_HUB.getPosition().getX()) { // if in middle zone
				if (robotPose.getY() > TurretTarget.BLUE_HUB.getPosition().getY()) {
					turretTargetPose = TurretTarget.BLUE_AIMING_TOP_CORNER.getPosition();
				} else {
					turretTargetPose = TurretTarget.BLUE_AIMING_BOTTOM_CORNER.getPosition();
				}

			} else {
				if (robotPose.getY() < TurretTarget.BLUE_HUB.getPosition().getY()) {
					turretTargetPose = TurretTarget.BLUE_FAR_TOP_CORNER.getPosition();
				} else {
					turretTargetPose = TurretTarget.BLUE_FAR_BOTTOM_CORNER.getPosition();
				}
			}
		}
	}

	/*
	RED_FAR_TOP_CORNER(new Translation2d(6.0, 6.0)), // 6.5
	BLUE_FAR_TOP_CORNER(new Translation2d(10.5, 6.0)),
	RED_FAR_BOTTOM_CORNER(new Translation2d(6.0, 2.0)), // 1.5
	BLUE_FAR_BOTTOM_CORNER(new Translation2d(10.5, 2.0)); */

	/**
	 * Calculates turret angle and sends position + velocity to the Kraken. Called at 100Hz via
	 * addPeriodic in Robot.java.
	 *
	 * <p>dt = TURRET_UPDATE_DT (0.010s) because this runs at 100Hz. The velocity feedforward lets the
	 * Kraken's 1kHz PID interpolate between our 10ms Java updates instead of waiting for error to
	 * grow.
	 *
	 * <p>All Logger calls have been moved to periodic() (50Hz, main thread) to avoid
	 * BufferOverflowException in the Notifier thread. Values are cached in fields above.
	 */
	public void updateTurretAngle() {
		Pose2d robotPose = driveRef.getPose();

		double controllerOmega = driverController.getRawAxis(4) * 0.15;

		Translation2d turretOffsetField = TURRET_OFFSET_ROBOT.rotateBy(robotPose.getRotation());
		Translation2d turretPos = robotPose.getTranslation().plus(turretOffsetField);

		ChassisSpeeds fieldSpeeds =
				ChassisSpeeds.fromRobotRelativeSpeeds(driveRef.getChassisSpeeds(), robotPose.getRotation());

		Translation2d virtualGoal = calculateVirtualGoal(turretPos, fieldSpeeds);

		double deltaX = virtualGoal.getX() - turretPos.getX();
		double deltaY = virtualGoal.getY() - turretPos.getY();
		double fieldAngleRad = Math.atan2(deltaY, deltaX);

		double robotHeadingRad = robotPose.getRotation().getRadians();
		robotHeadingRad = Math.atan2(Math.sin(robotHeadingRad), Math.cos(robotHeadingRad));

		double turretAngleRad = fieldAngleRad - robotHeadingRad;
		turretAngleRad = Math.atan2(Math.sin(turretAngleRad), Math.cos(turretAngleRad));

		double turretRangeCenter =
				(RobotConstants.Shooter.TURRET_RADIANS_MAX + RobotConstants.Shooter.TURRET_RADIANS_MIN)
						/ 2.0;
		while (turretAngleRad - turretRangeCenter > Math.PI) turretAngleRad -= 2 * Math.PI;
		while (turretAngleRad - turretRangeCenter < -Math.PI) turretAngleRad += 2 * Math.PI;

		// Differentiate the position setpoint to get velocity feedforward
		// dt = 0.010s because addPeriodic runs at 100Hz for now
		double turretSetpointVelocityRadPerSec =
				(turretAngleRad - previousTurretAngleRad) / TURRET_UPDATE_DT;
		previousTurretAngleRad = turretAngleRad;

		// Send both position AND velocity — Kraken uses kV*velocity as feedforward at 1kHz

		double turretAngleController = turretAngleRad + controllerOmega;

		shooterRef.setTurretPosition(turretAngleController, turretSetpointVelocityRadPerSec);

		// Cache for periodic() logging
		cache_turretPositionPose = new Pose2d(turretPos, new Rotation2d(fieldAngleRad));
		cache_actualGoalPose = new Pose2d(turretTargetPose, new Rotation2d());
		cache_virtualGoalPose = new Pose2d(virtualGoal, new Rotation2d());
		cache_actualAimPose =
				new Pose2d(turretPos, new Rotation2d(shooterRef.getTurretPosition() + robotHeadingRad));
		cache_targetAimPose = new Pose2d(turretPos, new Rotation2d(fieldAngleRad));
		cache_rangeCenter = turretRangeCenter;
		cache_rawTurretTarget = turretAngleRad;
		cache_targetAngleRobotRel = Math.toDegrees(turretAngleRad);
		cache_targetAngleFieldRel = Math.toDegrees(fieldAngleRad);
		cache_currentAngle = shooterRef.getTurretPosition();
		cache_turretDeltaDegrees = Math.toDegrees(turretAngleRad - shooterRef.getTurretPosition());
		cache_setpointVelocity = turretSetpointVelocityRadPerSec;
	}

	/**
	 * Sets shooter RPM based on distance to virtual goal. Called at 100Hz via addPeriodic in
	 * Robot.java.
	 */
	public void updateShooterVelocity() {
		Pose2d robotPose = driveRef.getPose();

		Translation2d turretOffsetField = TURRET_OFFSET_ROBOT.rotateBy(robotPose.getRotation());
		Translation2d turretPos = robotPose.getTranslation().plus(turretOffsetField);

		ChassisSpeeds fieldSpeeds =
				ChassisSpeeds.fromRobotRelativeSpeeds(driveRef.getChassisSpeeds(), robotPose.getRotation());

		Translation2d virtualGoal = calculateVirtualGoal(turretPos, fieldSpeeds);

		double distance = turretPos.getDistance(virtualGoal);
		double targetRPM = shooterRPMTable.get(distance);

		shooterRef.setShooterVelocities(targetRPM);

		// Cache for periodic() logging
		cache_shooterDistance = distance;
		cache_shooterTargetRPM = targetRPM;
		cache_shooterAverageRPM = shooterRef.getAverageShooterVelocity();
		cache_shotLine =
				new Pose2d[] {
					new Pose2d(turretPos, new Rotation2d()), new Pose2d(virtualGoal, new Rotation2d())
				};
	}

	// ====================================================================
	// Virtual Goal
	// ====================================================================

	private Translation2d calculateVirtualGoal(Translation2d turretPos, ChassisSpeeds fieldSpeeds) {
		double distanceToTarget = turretPos.getDistance(turretTargetPose);
		double targetRPM = shooterRPMTable.get(distanceToTarget);

		double wheelCircumference = Math.PI * RobotConstants.Shooter.SHOOTER_WHEEL_DIAMETER_METERS;
		double wheelSurfaceSpeed = (targetRPM / 60.0) * wheelCircumference;
		double exitVelocity = wheelSurfaceSpeed * RobotConstants.Shooter.SHOOTER_EFFICIENCY_FACTOR;

		double horizontalVelocity =
				exitVelocity * Math.cos(RobotConstants.Shooter.SHOOTER_ANGLE_RADIANS);

		double flightTime = distanceToTarget / horizontalVelocity;

		double displacementX =
				flightTime * fieldSpeeds.vxMetersPerSecond * RobotConstants.Shooter.SHOOTER_Y_COMP;
		double displacementY =
				flightTime * fieldSpeeds.vyMetersPerSecond * RobotConstants.Shooter.SHOOTER_Y_COMP;

		Translation2d virtualGoal =
				new Translation2d(
						turretTargetPose.getX() - displacementX, turretTargetPose.getY() - displacementY);

		double compensationOffset = turretTargetPose.getDistance(virtualGoal);

		// Cache for periodic() logging
		cache_motionComp_actualGoal = new Pose2d(turretTargetPose, new Rotation2d());
		cache_motionComp_virtualGoal = new Pose2d(virtualGoal, new Rotation2d());
		cache_motionComp_distToActual = distanceToTarget;
		cache_motionComp_targetRPM = targetRPM;
		cache_motionComp_exitVelocity = exitVelocity;
		cache_motionComp_horizVelocity = horizontalVelocity;
		cache_motionComp_flightTime = flightTime;
		cache_motionComp_robotVelX = fieldSpeeds.vxMetersPerSecond;
		cache_motionComp_robotVelY = fieldSpeeds.vyMetersPerSecond;
		cache_motionComp_displacementX = displacementX;
		cache_motionComp_displacementY = displacementY;
		cache_motionComp_compOffset = compensationOffset;

		return virtualGoal;
	}

	// ====================================================================
	// Shift / Game Data
	// ====================================================================

	public void findShift() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.length() == 0) return;

		switch (gameData.charAt(0)) {
			case 'B':
				if (!isRedGlobal) {
					SmartDashboard.putBoolean("Timekeeper/IsHubOn", false);
					SmartDashboard.putNumber("Timerkeeper/Countdown", 12);
				}
				break;
			case 'R':
				break;
			default:
				break;
		}
	}

	public void setContollerRumble() {
		if (shooterRef.isTurretAtTarget()) {
			driverController.setRumble(RumbleType.kBothRumble, 0.0);
		} else {
			driverController.setRumble(RumbleType.kBothRumble, 0.7);
		}
	}

	// ====================================================================
	// Getters & Setters
	// ====================================================================

	public void setRobotState(RobotState newRobotState) {
		currentRobotState = newRobotState;
	}

	public void setTurretTarget(Translation2d newTarget) {
		turretTargetPose = newTarget;
	}

	public Translation2d getTurretTarget() {
		return turretTargetPose;
	}
}
