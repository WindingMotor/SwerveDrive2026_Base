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
		INTAKE_IN,
		CLIMB_UP,
		CLIMB_DOWN,
		CLIMB_STOP,
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

	// Evaluated dynamically every loop — DS may not be connected at construction time!
	private boolean isRedGlobal =
			DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

	// ====================================================================
	// Constants
	// ====================================================================

	private final double INTAKE_MAX_EXTENSION_METERS = 11.5;

	private final InterpolatingDoubleTreeMap shooterRPMTable;

	// Turret offset from robot center (in robot frame)
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
			Drive driveRef) {
		this.indexerRef = indexerRef;
		this.intakeRef = intakeRef;
		this.shooterRef = shooterRef;
		this.driveRef = driveRef;

		// Initialize shooter RPM lookup table from constants
		shooterRPMTable = new InterpolatingDoubleTreeMap();
		for (double[] dataPoint : RobotConstants.Shooter.SHOOTER_RPM_DATA) {
			shooterRPMTable.put(dataPoint[0], dataPoint[1]);
		}

		timekeeper = new Timekeeper();
	}

	// ====================================================================
	// Periodic
	// ====================================================================

	@Override
	public void periodic() {
		Logger.recordOutput("Superstructure/RobotState", currentRobotState.toString());
		Logger.recordOutput("Superstructure/ActivelyShooting", activelyShooting);
		Logger.recordOutput("Superstructure/ActivelyReady", activelyReady);

		runStateMachine();

		updateTurretTarget();
		updateTurretAngle();
		updateShooterVelocity();

		timekeeper.update();
	}

	// ====================================================================
	// State Machine
	// ====================================================================

	// +12V is full forward | 0V is nothing | -12V is full reverse
	private void runStateMachine() {
		switch (currentRobotState) {
			case IDLE:
				indexerRef.setSpinnerVoltage(0.0);
				indexerRef.setKickerVoltage(0.0);
				intakeRef.setIntakeVoltage(0.0);
				intakeRef.setSliderPosition(0.0);
				shooterRef.setShooterVelocities(0.0);
				break;

			case SHOOTING:
				if (shooterRef.isTurretAtTarget()) {
					indexerRef.setSpinnerVoltage(12.0);
					indexerRef.setKickerVoltage(10.0);
				} else {
					indexerRef.setSpinnerVoltage(0.0);
					indexerRef.setKickerVoltage(0.0);
				}
				// FOR SIMULATION ONLY
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
				intakeRef.setIntakeVoltage(9.0);
				intakeRef.setSliderPosition(INTAKE_MAX_EXTENSION_METERS);
				break;

			case INTAKE_IN:
				intakeRef.setSliderPosition(0.0);
				intakeRef.setIntakeVoltage(0.0);
				break;

			case CLIMB_UP:
				indexerRef.setClimbVoltage(5.0);
				break;

			case CLIMB_DOWN:
				indexerRef.setClimbVoltage(-5.0);
				break;

			case CLIMB_STOP:
				indexerRef.setClimbVoltage(0.0);
				break;
		}
	}

	// ====================================================================
	// Turret & Shooter Updates
	// ====================================================================

	/**
	 * Selects the correct turret target position based on alliance and robot pose. Evaluated every
	 * loop so the alliance is read dynamically after DS connects.
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
	 * Calculates and sets the turret angle to compensate for robot motion. Uses the virtual goal
	 * method to account for shot flight time.
	 */
	private void updateTurretAngle() {
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

		// Normalize to [-π, π]
		turretAngleRad = Math.atan2(Math.sin(turretAngleRad), Math.cos(turretAngleRad));

		// Unwrap toward center of turret's valid range
		double turretRangeCenter =
				(RobotConstants.Shooter.TURRET_RADIANS_MAX + RobotConstants.Shooter.TURRET_RADIANS_MIN)
						/ 2.0;
		while (turretAngleRad - turretRangeCenter > Math.PI) turretAngleRad -= 2 * Math.PI;
		while (turretAngleRad - turretRangeCenter < -Math.PI) turretAngleRad += 2 * Math.PI;

		shooterRef.setTurretPosition(turretAngleRad);

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
	}

	/**
	 * Calculates and sets the shooter wheel velocity based on distance to target. Accounts for
	 * virtual goal position when the robot is moving.
	 */
	private void updateShooterVelocity() {
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
					new Pose2d(turretPos, new Rotation2d()), // start
					new Pose2d(virtualGoal, new Rotation2d()) // end
				};

		Logger.recordOutput("Superstructure/Shooter/DistanceToVirtualGoal", distance);
		Logger.recordOutput("Superstructure/Shooter/TargetRPM", targetRPM);
		Logger.recordOutput(
				"Superstructure/Shooter/AverageRPM", shooterRef.getAverageShooterVelocity());
		Logger.recordOutput("Superstructure/Shooter/DistanceToVirtualGoalLine", shotLine);
	}

	/**
	 * Calculates the virtual goal position, compensating for robot motion during shot flight time.
	 * Uses a physics-based flight time calculation for a fixed-angle shooter.
	 *
	 * @param turretPos Current turret position in field frame
	 * @param fieldSpeeds Robot velocity in field frame (m/s)
	 * @return Virtual goal position accounting for motion compensation
	 */
	private Translation2d calculateVirtualGoal(Translation2d turretPos, ChassisSpeeds fieldSpeeds) {
		double distanceToTarget = turretPos.getDistance(turretTargetPose);
		double targetRPM = shooterRPMTable.get(distanceToTarget);

		// Convert RPM to exit velocity (m/s)
		double wheelCircumference = Math.PI * RobotConstants.Shooter.SHOOTER_WHEEL_DIAMETER_METERS;
		double wheelSurfaceSpeed = (targetRPM / 60.0) * wheelCircumference;
		double exitVelocity = wheelSurfaceSpeed * RobotConstants.Shooter.SHOOTER_EFFICIENCY_FACTOR;

		// Horizontal velocity component (fixed launch angle)
		double horizontalVelocity =
				exitVelocity * Math.cos(RobotConstants.Shooter.SHOOTER_ANGLE_RADIANS);

		// Flight time: distance / horizontal velocity
		double flightTime = distanceToTarget / horizontalVelocity;

		// Robot displacement during flight
		double displacementX = flightTime * fieldSpeeds.vxMetersPerSecond;
		double displacementY = flightTime * fieldSpeeds.vyMetersPerSecond;

		// Subtract displacement — aim "behind" direction of motion
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
	// Shift / Game Data
	// ====================================================================

	public void findShift() {
		String gameData = DriverStation.getGameSpecificMessage();
		if (gameData.length() == 0) return;

		switch (gameData.charAt(0)) {
			case 'B':
				if (!isRedGlobal) { // second shift
					SmartDashboard.putBoolean("Timekeeper/IsHubOn", false);
					SmartDashboard.putNumber("Timerkeeper/Countdown", 12);
				} else { // first shift
				}
				break;
			case 'R':
				if (isRedGlobal) { // second shift
				} else { // first shift
				}
				break;
			default:
				// Corrupt data
				break;
		}
	}

	// ====================================================================
	// Getters & Setters
	// ====================================================================

	public void setRobotState(RobotState newRobotState) {
		currentRobotState = newRobotState;
	}

	/** Manually override the turret target. Useful for autonomous or dynamic target switching. */
	public void setTurretTarget(Translation2d newTarget) {
		turretTargetPose = newTarget;
		Logger.recordOutput("Superstructure/TargetPoseUpdated", newTarget);
	}

	public Translation2d getTurretTarget() {
		return turretTargetPose;
	}
}
