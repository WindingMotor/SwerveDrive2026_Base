// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.lib.windingmotor.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.LIB_DriveConstants;
import frc.robot.constants.LIB_PoseConstants;
import frc.robot.constants.LIB_VisionConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotConstants.RobotMode;
import frc.robot.lib.windingmotor.drive.gyro.GyroInputsAutoLogged;
import frc.robot.lib.windingmotor.drive.gyro.IO_GyroBase;
import frc.robot.lib.windingmotor.drive.module.IO_ModuleBase;
import frc.robot.lib.windingmotor.drive.module.Module;
import frc.robot.lib.windingmotor.util.auto.LocalADStarAK;
import frc.robot.lib.windingmotor.util.phoenix.PhoenixOdometryThread;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Swerve Drive Subsystem - The Core of Robot Mobility
 *
 * <p>This subsystem implements a fully-featured swerve drive with: - High-frequency odometry (250Hz
 * on CAN FD, 100Hz otherwise) - Vision integration for pose correction - PathPlanner autonomous
 * path following - SysId characterization routines - Real-time pose estimation with gyro fallback -
 * Comprehensive logging and telemetry
 *
 * <p>The architecture separates concerns into: 1. IO Layer (IO_ModuleBase, IO_GyroBase) - Hardware
 * abstraction 2. Module Layer (Module) - Individual swerve module control 3. Drive Layer (Drive) -
 * Coordination and odometry
 *
 * <p>Odometry is processed on a separate thread (PhoenixOdometryThread) to achieve high-frequency
 * updates without being blocked by the main 20ms loop, enabling accurate pose estimation even
 * during aggressive maneuvers.
 */
public class Drive extends SubsystemBase {

	// ====================================================================
	// SECTION 1: Constants and Configuration
	// ====================================================================

	/** AprilTag field layout for vision-based localization. */
	private final AprilTagFieldLayout aprilTagFieldLayout =
			AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

	// TODO: Change to 2026 field when tag layout comes out.

	/**
	 * Odometry update frequency (Hz) - depends on CAN FD capability. CAN FD networks can handle 250Hz
	 * updates, while classic CAN is limited to 100Hz. Higher frequency = more accurate pose
	 * estimation during dynamic movements.
	 */
	public static final double ODOMETRY_FREQUENCY =
			new CANBus(LIB_DriveConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

	/**
	 * Drive base radius (meters) - the distance from robot center to furthest module. Calculated from
	 * module positions to ensure no module exceeds this radius during rotation. Used for angular
	 * velocity calculations and constraint enforcement.
	 */
	public static final double DRIVE_BASE_RADIUS =
			Math.max(
					Math.max(
							Math.hypot(
									LIB_DriveConstants.FrontLeft.LocationX, LIB_DriveConstants.FrontLeft.LocationY),
							Math.hypot(
									LIB_DriveConstants.FrontRight.LocationX,
									LIB_DriveConstants.FrontRight.LocationY)),
					Math.max(
							Math.hypot(
									LIB_DriveConstants.BackLeft.LocationX, LIB_DriveConstants.BackLeft.LocationY),
							Math.hypot(
									LIB_DriveConstants.BackRight.LocationX, LIB_DriveConstants.BackRight.LocationY)));

	// ====================================================================
	// SECTION 2: PathPlanner Configuration
	// ====================================================================

	/** Robot physical properties for PathPlanner trajectory generation. */

	/** PathPlanner robot configuration combining all physical properties. */
	private static final RobotConfig PP_CONFIG =
			new RobotConfig(
					LIB_DriveConstants.ROBOT_MASS_KG,
					LIB_DriveConstants.ROBOT_MOI,
					new ModuleConfig(
							LIB_DriveConstants.FrontLeft.WheelRadius,
							LIB_DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
							LIB_DriveConstants.WHEEL_COF,
							DCMotor.getKrakenX60Foc(1)
									.withReduction(LIB_DriveConstants.FrontLeft.DriveMotorGearRatio),
							LIB_DriveConstants.FrontLeft.SlipCurrent,
							1),
					getModuleTranslations());

	// ====================================================================
	// SECTION 3: Core Subsystem Components
	// ====================================================================

	/** Lock to prevent concurrent access to odometry data during updates. */
	public static final Lock odometryLock = new ReentrantLock();

	/** Gyro IO interface (real, sim, or replay). */
	private final IO_GyroBase gyroIO;

	/** Auto-logged gyro inputs for AdvantageKit. */
	private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();

	/** Four swerve modules (FL, FR, BL, BR). */
	private final Module[] modules = new Module[4];

	/** SysId routine for drive characterization. */
	private final SysIdRoutine sysId;

	/** Alert for disconnected gyro (warns if using kinematics fallback). */
	private final Alert gyroDisconnectedAlert =
			new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

	// ====================================================================
	// SECTION 4: Kinematics and Pose Estimation
	// ====================================================================

	/** Swerve kinematics calculator (converts between chassis speeds and module states). */
	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

	/** Raw gyro rotation (may be from real gyro or calculated from kinematics). */
	private Rotation2d rawGyroRotation = new Rotation2d();

	/** Last module positions for delta tracking. */
	private SwerveModulePosition[] lastModulePositions =
			new SwerveModulePosition[] {
				new SwerveModulePosition(),
				new SwerveModulePosition(),
				new SwerveModulePosition(),
				new SwerveModulePosition()
			};

	/** Swerve pose estimator combining gyro, module odometry, and vision. */
	private SwerveDrivePoseEstimator poseEstimator =
			new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

	// ====================================================================
	// SECTION 5: Vision and Localization
	// ====================================================================

	/** Cache of closest AprilTag ID and distance for auto-alignment. */
	private Pair<Integer, Double> closestTagData = Pair.of(-1, Double.MAX_VALUE);

	/** Helper for pose alignment calculations. */
	public LIB_PoseConstants poseAllignment = new LIB_PoseConstants();

	// ====================================================================
	// SECTION 6: Constructor and Initialization
	// ====================================================================

	/**
	 * Constructs the Drive subsystem with all required components.
	 *
	 * @param gyroIO Gyro IO implementation
	 * @param flModuleIO Front-left module IO
	 * @param frModuleIO Front-right module IO
	 * @param blModuleIO Back-left module IO
	 * @param brModuleIO Back-right module IO
	 */
	public Drive(
			IO_GyroBase gyroIO,
			IO_ModuleBase flModuleIO,
			IO_ModuleBase frModuleIO,
			IO_ModuleBase blModuleIO,
			IO_ModuleBase brModuleIO) {
		this.gyroIO = gyroIO;

		// Initialize modules with their IO implementations
		modules[0] = new Module(flModuleIO, 0, LIB_DriveConstants.FrontLeft);
		modules[1] = new Module(frModuleIO, 1, LIB_DriveConstants.FrontRight);
		modules[2] = new Module(blModuleIO, 2, LIB_DriveConstants.BackLeft);
		modules[3] = new Module(brModuleIO, 3, LIB_DriveConstants.BackRight);

		// Report swerve drive usage to WPILib (for telemetry)
		HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

		// Start the high-frequency odometry thread
		PhoenixOdometryThread.getInstance().start();

		// Configure PathPlanner for autonomous path following
		configurePathPlanner();

		// Configure SysId for characterization
		sysId =
				new SysIdRoutine(
						new SysIdRoutine.Config(
								null, // Default ramp rate (1 V/s)
								null, // Default step voltage (7 V)
								null, // Default timeout (10 s)
								(state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
						new SysIdRoutine.Mechanism(
								(voltage) -> runCharacterization(voltage.in(Volts)), // Voltage consumer
								null, // No log consumer (logging handled by subsystem)
								this)); // Requirements
	}

	/**
	 * Configures PathPlanner for autonomous path following.
	 *
	 * <p>Sets up: - Pose suppliers for starting pose and pose updates - Chassis speeds supplier for
	 * velocity feedback - Velocity consumer for motor commands - Holonomic drive controller with PID
	 * constants - Pathfinding for dynamic obstacle avoidance - Logging callbacks for telemetry
	 */
	private void configurePathPlanner() {
		AutoBuilder.configure(
				this::getPose, // Current pose supplier
				this::setPose, // Pose reset consumer
				this::getChassisSpeeds, // Current chassis speeds supplier
				this::runVelocity, // Velocity command consumer
				new PPHolonomicDriveController(
						new PIDConstants(4.45, 0.0, 0.0), // Translation PID
						new PIDConstants(5.3, 0.0, 0.0)), // Rotation PID
				PP_CONFIG, // Robot configuration
				() ->
						DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Flipping condition
				this);

		// Configure custom pathfinding for dynamic obstacle avoidance
		Pathfinding.setPathfinder(new LocalADStarAK());

		// Log active path for visualization
		PathPlannerLogging.setLogActivePathCallback(
				(activePath) -> {
					Logger.recordOutput(
							"Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
				});

		// Log target pose for visualization
		PathPlannerLogging.setLogTargetPoseCallback(
				(targetPose) -> {
					Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
				});
	}

	// ====================================================================
	// SECTION 7: Periodic Update Loop
	// ====================================================================

	@Override
	public void periodic() {
		// Lock odometry to prevent concurrent access during high-frequency updates
		odometryLock.lock();
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);

		// Update all modules
		for (var module : modules) {
			module.periodic();
		}
		odometryLock.unlock();

		// Stop modules when disabled to prevent unwanted movement
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
		}

		// Log empty setpoints when disabled to avoid dashboard confusion
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
		}

		// Process odometry at high frequency for accuracy
		processOdometry();

		// Update gyro alert (not in sim mode since sim doesn't have a real gyro)
		gyroDisconnectedAlert.set(!gyroInputs.connected && RobotConstants.ROBOT_MODE != RobotMode.SIM);

		// Update camera positions for visualization
		updateCameraPositions();

		// Update closest tag data for auto-alignment
		closestTagData = getClosestAprilTagID();
	}

	/**
	 * Processes odometry at high frequency (250Hz or 100Hz depending on CAN network).
	 *
	 * <p>Unlike the main 20ms loop, this method processes every odometry sample from the motor
	 * controllers, providing much higher resolution for pose estimation. This is critical for
	 * accurate odometry during fast movements and rotations.
	 *
	 * <p>The method: 1. Reads module positions and deltas from each sampled timestamp 2. Updates gyro
	 * angle (real or calculated from kinematics) 3. Applies the pose update with the exact timestamp
	 */
	private void processOdometry() {
		// Get timestamps from first module (all modules are sampled synchronously)
		double[] sampleTimestamps = modules[0].getOdometryTimestamps();
		int sampleCount = sampleTimestamps.length;

		// Process each sample
		for (int i = 0; i < sampleCount; i++) {
			// Read positions and calculate deltas for each module
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
			for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
				modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
				// Calculate delta from last position
				moduleDeltas[moduleIndex] =
						new SwerveModulePosition(
								modulePositions[moduleIndex].distanceMeters
										- lastModulePositions[moduleIndex].distanceMeters,
								modulePositions[moduleIndex].angle);
				// Update last position for next delta calculation
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}

			// Update gyro angle
			if (gyroInputs.connected) {
				// Use real gyro data when available
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				// Fall back to calculating rotation from module deltas
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

			// Apply the pose update with the exact timestamp
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
		}
	}

	// ====================================================================
	// SECTION 8: Vision Integration
	// ====================================================================

	/**
	 * Updates camera positions for AdvantageScope visualization.
	 *
	 * <p>Transforms the camera positions from robot-relative to field-relative coordinates using the
	 * current robot pose. This allows viewing camera frustums in 3D visualization.
	 */
	private void updateCameraPositions() {
		Pose3d robotPose3d = new Pose3d(getPose());

		// Transform all camera poses to global coordinates
		int cameraCount = 3;
		Pose3d[] globalCameraPositions = new Pose3d[cameraCount];
		globalCameraPositions[0] = robotPose3d.transformBy(LIB_VisionConstants.robotToCamera0);
		globalCameraPositions[1] = robotPose3d.transformBy(LIB_VisionConstants.robotToCamera1);
		globalCameraPositions[2] = robotPose3d.transformBy(LIB_VisionConstants.robotToCamera2);
		// globalCameraPositions[3] = robotPose3d.transformBy(LIB_VisionConstants.robotToCamera3);

		// Log for visualization
		Logger.recordOutput("CameraPositions", globalCameraPositions);
	}

	// ====================================================================
	// SECTION 9: Control Methods
	// ====================================================================

	/**
	 * Runs the drive at the desired chassis speeds.
	 *
	 * <p>Converts chassis speeds to module states, desaturates wheel speeds to prevent commanding
	 * velocities beyond the robot's capability, and sends setpoints to modules.
	 *
	 * @param speeds Desired chassis speeds in meters/sec and radians/sec
	 */
	public void runVelocity(ChassisSpeeds speeds) {
		// Discretize continuous speeds to discrete time step (0.02s) for accurate control
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

		// Calculate module states from chassis speeds
		SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);

		// Desaturate wheel speeds to prevent commanding impossible velocities
		SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, LIB_DriveConstants.kSpeedAt12Volts);

		// Log unoptimized setpoints for debugging
		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

		// Send setpoints to modules (modules will optimize angles internally)
		for (int i = 0; i < 4; i++) {
			modules[i].runSetpoint(setpointStates[i]);
		}

		// Log optimized setpoints (modules may mutate the states during optimization)
		Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
	}

	/** Runs the drive in a straight line with the specified voltage output for characterization. */
	public void runCharacterization(double output) {
		// Apply the same voltage to all modules for straight-line characterization
		for (int i = 0; i < 4; i++) {
			modules[i].runCharacterization(output);
		}
	}

	/** Stops all module motion (sets velocity to zero). */
	public void stop() {
		runVelocity(new ChassisSpeeds());
	}

	/**
	 * Stops the drive and turns modules to X-formation to resist movement.
	 *
	 * <p>Modules are oriented toward the robot center, creating a mechanically locked configuration.
	 * The formation automatically resets when nonzero velocity is commanded.
	 */
	public void stopWithX() {
		// Calculate headings toward robot center for each module
		Rotation2d[] headings = new Rotation2d[4];
		for (int i = 0; i < 4; i++) {
			headings[i] = getModuleTranslations()[i].getAngle();
		}
		kinematics.resetHeadings(headings);
		stop();
	}

	// ====================================================================
	// SECTION 10: SysId Characterization
	// ====================================================================

	/**
	 * Returns a command to run a quasistatic test in the specified direction.
	 *
	 * <p>Quasistatic tests apply a slowly ramping voltage to characterize static friction and
	 * velocity feedforward terms. The test starts with a 1-second zero-voltage period to establish a
	 * baseline, then runs the SysId routine.
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0)) // 1 second at zero voltage
				.withTimeout(1.0)
				.andThen(sysId.quasistatic(direction));
	}

	/**
	 * Returns a command to run a dynamic test in the specified direction.
	 *
	 * <p>Dynamic tests apply a step voltage to characterize acceleration feedforward and feedback
	 * gains. The test starts with a 1-second zero-voltage period to establish a baseline, then runs
	 * the SysId routine.
	 */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return run(() -> runCharacterization(0.0)) // 1 second at zero voltage
				.withTimeout(1.0)
				.andThen(sysId.dynamic(direction));
	}

	// ====================================================================
	// SECTION 11: State and Telemetry
	// ====================================================================

	/** Returns measured module states (angles and velocities). */
	@AutoLogOutput(key = "SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	/** Returns module positions (angles and distances traveled). */
	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			positions[i] = modules[i].getPosition();
		}
		return positions;
	}

	/** Returns measured chassis speeds calculated from module states. */
	@AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	/** Returns wheel radius characterization positions (for wheel radius calibration). */
	public double[] getWheelRadiusCharacterizationPositions() {
		double[] values = new double[4];
		for (int i = 0; i < 4; i++) {
			values[i] = modules[i].getWheelRadiusCharacterizationPosition();
		}
		return values;
	}

	/** Returns average drive motor velocity for feedforward characterization. */
	public double getFFCharacterizationVelocity() {
		double output = 0.0;
		for (int i = 0; i < 4; i++) {
			output += modules[i].getFFCharacterizationVelocity() / 4.0;
		}
		return output;
	}

	/** Returns current estimated pose. */
	@AutoLogOutput(key = "Odometry/Robot")
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/** Returns current rotation from pose estimator. */
	public Rotation2d getRotation() {
		return getPose().getRotation();
	}

	/** Resets pose estimator to specified pose. */
	public void setPose(Pose2d pose) {
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	/**
	 * Adds a vision measurement to the pose estimator.
	 *
	 * <p>Vision measurements are weighted by their standard deviations. Lower std dev = more trust in
	 * the vision measurement.
	 */
	public void addVisionMeasurement(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs) {
		poseEstimator.addVisionMeasurement(
				visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
	}

	/** Returns maximum linear speed (m/s). */
	public double getMaxLinearSpeedMetersPerSec() {
		return LIB_DriveConstants.kSpeedAt12Volts.in(MetersPerSecond);
	}

	/** Returns maximum angular speed (rad/s) based on drive base radius. */
	public double getMaxAngularSpeedRadPerSec() {
		return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
	}

	/** Returns array of module position translations. */
	public static Translation2d[] getModuleTranslations() {
		return new Translation2d[] {
			new Translation2d(
					LIB_DriveConstants.FrontLeft.LocationX, LIB_DriveConstants.FrontLeft.LocationY),
			new Translation2d(
					LIB_DriveConstants.FrontRight.LocationX, LIB_DriveConstants.FrontRight.LocationY),
			new Translation2d(
					LIB_DriveConstants.BackLeft.LocationX, LIB_DriveConstants.BackLeft.LocationY),
			new Translation2d(
					LIB_DriveConstants.BackRight.LocationX, LIB_DriveConstants.BackRight.LocationY)
		};
	}

	// ====================================================================
	// SECTION 12: AprilTag Utilities
	// ====================================================================

	/**
	 * Finds the closest AprilTag to the robot's current position.
	 *
	 * <p>Checks all 22 AprilTags on the field and returns the ID and distance of the closest one.
	 * Used for auto-alignment and dynamic state selection.
	 *
	 * @return Pair of (tag ID, distance in meters)
	 */
	public Pair<Integer, Double> getClosestAprilTagID() {
		Pose2d currentPose = getPose();
		double minDistance = Double.MAX_VALUE;
		int closestTagId = -1;

		// Check all 22 possible AprilTags
		for (int tagId = 1; tagId <= 22; tagId++) {
			Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagId);
			if (tagPose.isPresent()) {
				double distance =
						currentPose.getTranslation().getDistance(tagPose.get().toPose2d().getTranslation());

				if (distance < minDistance) {
					minDistance = distance;
					closestTagId = tagId;
				}
			}
		}

		// Log for debugging and dashboards
		Logger.recordOutput("ClosestAprilTag/Id", closestTagId);
		Logger.recordOutput("ClosestAprilTag/Distance", minDistance);

		return Pair.of(closestTagId, minDistance);
	}

	/**
	 * Gets the field location of a specific AprilTag.
	 *
	 * @param id The AprilTag ID (1-22)
	 * @return Translation2d of the tag's position, or (0,0) if not found
	 */
	public Translation2d getApriltagLocation(int id) {
		if (id > 0) {
			Optional<Pose3d> pose3d = aprilTagFieldLayout.getTagPose(id);
			return new Translation2d(pose3d.get().getX(), pose3d.get().getY());
		} else {
			return new Translation2d();
		}
	}

	/** Returns the cached closest tag data from the last periodic cycle. */
	public Pair<Integer, Double> getRecentClosestTagData() {
		return closestTagData;
	}
}
