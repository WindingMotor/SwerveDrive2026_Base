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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotMode;
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
		BLUE_AIMING_TOP_CORNER(new Translation2d(4.0, 6.5)),
		RED_AIMING_TOP_CORNER(new Translation2d(12.5, 6.5)),
		BLUE_AIMING_BOTTOM_CORNER(new Translation2d(4.0, 1.5)),
		RED_AIMING_BOTTOM_CORNER(new Translation2d(12.5, 1.5));

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
		INTAKE_AUTO,
		INTAKE_IN,
		INTAKE_LIMP,
		CLIMB_TOP,
		CLIMB_BOTTOM,
		CLIMB_UP,
		CLIMB_DOWN,
		CLIMB_STOP,
		EJECT
	}

	// ====================================================================
	// Subsystem References
	// ====================================================================

	private SUB_Indexer indexerRef;
	private SUB_Intake intakeRef;
	private SUB_Shooter shooterRef;
	private Drive driveRef;

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

	// ====================================================================
	// Constants
	// ====================================================================

	private final double INTAKE_MAX_EXTENSION_METERS = 11.5;

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
			SUB_Indexer indexerRef, SUB_Intake intakeRef, SUB_Shooter shooterRef, Drive driveRef) {
		this.indexerRef = indexerRef;
		this.intakeRef = intakeRef;
		this.shooterRef = shooterRef;
		this.driveRef = driveRef;

		shooterRPMTable = new InterpolatingDoubleTreeMap();
		for (double[] dataPoint : RobotConstants.Shooter.SHOOTER_RPM_DATA) {
			shooterRPMTable.put(dataPoint[0], dataPoint[1]);
		}

		timekeeper = new Timekeeper();
	}

	// ====================================================================
	// Periodic — 50Hz main loop
	// Only the state machine lives here. Turret updates run at 100Hz
	// via addPeriodic in Robot.java and must NOT also run here.
	// ====================================================================

	@Override
	public void periodic() {
		Logger.recordOutput("Superstructure/RobotState", currentRobotState.toString());
		Logger.recordOutput("Superstructure/ActivelyShooting", activelyShooting);
		Logger.recordOutput("Superstructure/ActivelyReady", activelyReady);
		Logger.recordOutput("Superstructure/RobotPose", driveRef.getPose());

		// Setpoints are already sent at 100Hz by Robot.java's addPeriodic.
		// The state machine just gates the indexer/kicker — 50Hz is fine for that.
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
				shooterRef.setShooterVelocities(0.0);
				indexerRef.setClimbVoltage(0.0);
				break;

			case SHOOTING:
				if (shooterRef.isReadyToShoot()) {
					indexerRef.setSpinnerVoltage(12.0);
					indexerRef.setKickerVoltage(10.0);
					Logger.recordOutput("Superstructure/Turret/isTurretAtTarget", true);
				} else {
					indexerRef.setSpinnerVoltage(0.0);
					indexerRef.setKickerVoltage(0.0);
					Logger.recordOutput("Superstructure/Turret/isTurretAtTarget", false);
				}
				if (RobotConstants.ROBOT_MODE == RobotMode.SIM) {
					if (shooterRef.isShooterAtSpeed(shooterRef.getShooterVelocityRPMSetpoint(), 100.0)) {
						shooterRef.onShoot();
					}
				}
				break;

			case READY:
				indexerRef.setSpinnerVoltage(0.0);
				indexerRef.setKickerVoltage(0.0);
				break;

			case UNJAM:
				indexerRef.setSpinnerVoltage(-4.0);
				break;

			case INTAKE:
				intakeRef.setIntakeVoltage(10.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				//intakeRef.setSliderVoltage(8.0);
				break;

			case INTAKE_AUTO:
				intakeRef.setIntakeVoltage(12.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				//intakeRef.setSliderVoltage(8.0);
				break;

			case INTAKE_LIMP:
				intakeRef.setIntakeVoltage(2.0);
				intakeRef.setSliderVoltage(0.0);
				break;

			case EJECT:
				intakeRef.setIntakeVoltage(-10.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				//intakeRef.setSliderVoltage(8.0);
				break;

			case INTAKE_IN:
				//intakeRef.setSliderVoltage(-8.0);
				intakeRef.setSliderPosition(0.0);
				intakeRef.setIntakeVoltage(2.0);
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
	}

	// ====================================================================
	// Turret & Shooter Updates — PUBLIC, called at 100Hz by Robot.java
	// ====================================================================

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
			if (robotPose.getX() > TurretTarget.RED_HUB.getPosition().getX()) {
				turretTargetPose = TurretTarget.RED_HUB.getPosition();
			} else if (robotPose.getY() > TurretTarget.RED_HUB.getPosition().getY()) {
				turretTargetPose = TurretTarget.RED_AIMING_TOP_CORNER.getPosition();
			} else {
				turretTargetPose = TurretTarget.RED_AIMING_BOTTOM_CORNER.getPosition();
			}
		} else {
			if (robotPose.getX() < TurretTarget.BLUE_HUB.getPosition().getX()) {
				turretTargetPose = TurretTarget.BLUE_HUB.getPosition();
			} else if (robotPose.getY() > TurretTarget.BLUE_HUB.getPosition().getY()) {
				turretTargetPose = TurretTarget.BLUE_AIMING_TOP_CORNER.getPosition();
			} else {
				turretTargetPose = TurretTarget.BLUE_AIMING_BOTTOM_CORNER.getPosition();
			}
		}
	}

	/**
	 * Calculates turret angle and sends position + velocity to the Kraken. Called at 100Hz via
	 * addPeriodic in Robot.java.
	 *
	 * <p>dt = TURRET_UPDATE_DT (0.010s) because this runs at 100Hz. The velocity feedforward lets the
	 * Kraken's 1kHz PID interpolate between our 10ms Java updates instead of waiting for error to
	 * grow.
	 */
	public void updateTurretAngle() {
		Pose2d robotPose = driveRef.getPose();

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
		// dt = 0.010s because addPeriodic runs at 100Hz
		double turretSetpointVelocityRadPerSec =
				(turretAngleRad - previousTurretAngleRad) / TURRET_UPDATE_DT;
		previousTurretAngleRad = turretAngleRad;

		// Send both position AND velocity — Kraken uses kV*velocity as feedforward at 1kHz
		shooterRef.setTurretPosition(turretAngleRad, turretSetpointVelocityRadPerSec);

		Logger.recordOutput(
				"Superstructure/Turret/TurretPosition",
				new Pose2d(turretPos, new Rotation2d(fieldAngleRad)));
		Logger.recordOutput(
				"Superstructure/Turret/ActualGoal", new Pose2d(turretTargetPose, new Rotation2d()));
		Logger.recordOutput(
				"Superstructure/Turret/VirtualGoal", new Pose2d(virtualGoal, new Rotation2d()));
		Logger.recordOutput(
				"Superstructure/Turret/ActualAim",
				new Pose2d(turretPos, new Rotation2d(shooterRef.getTurretPosition() + robotHeadingRad)));
		Logger.recordOutput(
				"Superstructure/Turret/TargetAim", new Pose2d(turretPos, new Rotation2d(fieldAngleRad)));
		Logger.recordOutput("Superstructure/Turret/RangeCenter", turretRangeCenter);
		Logger.recordOutput("Superstructure/Turret/RawTurretTarget", turretAngleRad);
		Logger.recordOutput(
				"Superstructure/Turret/TargetAngleRobotRelative", Math.toDegrees(turretAngleRad));
		Logger.recordOutput(
				"Superstructure/Turret/TargetAngleFieldRelative", Math.toDegrees(fieldAngleRad));
		Logger.recordOutput("Superstructure/Turret/CurrentAngle", shooterRef.getTurretPosition());
		Logger.recordOutput(
				"Superstructure/Turret/turretDeltaDegrees",
				Math.toDegrees(turretAngleRad - shooterRef.getTurretPosition()));
		Logger.recordOutput(
				"Superstructure/Turret/SetpointVelocityRadPerSec", turretSetpointVelocityRadPerSec);
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

		Pose2d[] shotLine =
				new Pose2d[] {
					new Pose2d(turretPos, new Rotation2d()), new Pose2d(virtualGoal, new Rotation2d())
				};

		Logger.recordOutput("Superstructure/Shooter/DistanceToVirtualGoal", distance);
		Logger.recordOutput("Superstructure/Shooter/TargetRPM", targetRPM);
		Logger.recordOutput(
				"Superstructure/Shooter/AverageRPM", shooterRef.getAverageShooterVelocity());
		Logger.recordOutput("Superstructure/Shooter/DistanceToVirtualGoalLine", shotLine);
	}

	// ====================================================================
	// Virtual Goal (unchanged)
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

		double displacementX = flightTime * fieldSpeeds.vxMetersPerSecond;
		double displacementY = flightTime * fieldSpeeds.vyMetersPerSecond;

		Translation2d virtualGoal =
				new Translation2d(
						turretTargetPose.getX() - displacementX, turretTargetPose.getY() - displacementY);

		double compensationOffset = turretTargetPose.getDistance(virtualGoal);

		Logger.recordOutput(
				"Superstructure/MotionComp/ActualGoal", new Pose2d(turretTargetPose, new Rotation2d()));
		Logger.recordOutput(
				"Superstructure/MotionComp/VirtualGoal", new Pose2d(virtualGoal, new Rotation2d()));
		Logger.recordOutput("Superstructure/MotionComp/DistanceToActualGoal", distanceToTarget);
		Logger.recordOutput("Superstructure/MotionComp/TargetRPM", targetRPM);
		Logger.recordOutput("Superstructure/MotionComp/ExitVelocityMPS", exitVelocity);
		Logger.recordOutput("Superstructure/MotionComp/HorizontalVelocityMPS", horizontalVelocity);
		Logger.recordOutput("Superstructure/MotionComp/FlightTimeSeconds", flightTime);
		Logger.recordOutput("Superstructure/MotionComp/RobotVelocityX", fieldSpeeds.vxMetersPerSecond);
		Logger.recordOutput("Superstructure/MotionComp/RobotVelocityY", fieldSpeeds.vyMetersPerSecond);
		Logger.recordOutput("Superstructure/MotionComp/DisplacementX", displacementX);
		Logger.recordOutput("Superstructure/MotionComp/DisplacementY", displacementY);
		Logger.recordOutput("Superstructure/MotionComp/CompensationOffsetMeters", compensationOffset);

		return virtualGoal;
	}

	// ====================================================================
	// Shift / Game Data (unchanged)
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

	// ====================================================================
	// Getters & Setters
	// ====================================================================

	public void setRobotState(RobotState newRobotState) {
		currentRobotState = newRobotState;
	}

	public void setTurretTarget(Translation2d newTarget) {
		turretTargetPose = newTarget;
		Logger.recordOutput("Superstructure/TargetPoseUpdated", newTarget);
	}

	public Translation2d getTurretTarget() {
		return turretTargetPose;
	}
}
