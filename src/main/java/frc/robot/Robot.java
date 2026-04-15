// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.lib.windingmotor.drive.Drive;
import frc.robot.lib.windingmotor.util.fieldsim.SimulationManager;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Main robot class that extends AdvantageKit's LoggedRobot to enable comprehensive logging and
 * replay.
 *
 * <p>This class orchestrates the entire robot lifecycle, from initialization through autonomous,
 * teleop, and test modes. It configures AdvantageKit's data receivers based on the robot mode
 * (REAL, SIM, or REPLAY) and manages the WPILib CommandScheduler.
 *
 * <p>Key responsibilities: - Initialize logging infrastructure with appropriate data receivers -
 * Record build metadata for debugging and version tracking - Manage the CommandScheduler lifecycle
 * across all robot modes - Handle the transition between autonomous and teleop modes - Configure
 * system properties like watchdog timeout and brownout voltage
 *
 * <p>The three operational modes are: - REAL: Logs to USB and NetworkTables, enables power
 * distribution monitoring - SIM: Publishes to NetworkTables only, runs at maximum speed without
 * timing - REPLAY: Reads from a log file and writes outputs to a new log for verification
 */
public class Robot extends LoggedRobot {
	/** Warning threshold for loop overruns (seconds). */
	private static final double loopOverrunWarningTimeout = 0.2;

	/** The autonomous command to run during autonomous period. */
	private Command m_autonomousCommand;

	/** The robot container that holds all subsystems and commands. */
	private final RobotContainer m_robotContainer;

	// Notifier runs the turret update loop at 100Hz on a dedicated thread
	private final Notifier turretUpdateNotifier;

	/**
	 * Constructs the Robot and initializes logging infrastructure.
	 *
	 * <p>Records build metadata (Git SHA, branch, date) and configures AdvantageKit data receivers
	 * based on the current robot mode. Also adjusts the watchdog timeout to reduce spam from
	 * occasional loop overruns.
	 */
	@SuppressWarnings("resource")
	public Robot() {
		m_robotContainer = new RobotContainer();

		Logger.recordMetadata("Maven Name", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("Build Date", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("ProjectName", "SwerveDrive2025");
		Logger.recordMetadata("Robot Mode", RobotConstants.ROBOT_MODE.toString());
		Logger.recordMetadata("Git Branch", BuildConstants.GIT_BRANCH);
		Logger.recordMetadata(
				"Authors", "Benjamin L & (WindingMotor) Isaac S & FRC 2106 Junkyard Dogs");

		switch (RobotConstants.ROBOT_MODE) {
			case REAL:
				Logger.addDataReceiver(new WPILOGWriter("/u/usblogs"));
				// Logger.addDataReceiver(new NT4Publisher());
				new PowerDistribution(1, ModuleType.kRev);
				break;
			case SIM:
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case REPLAY:
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;
		}

		Logger.start();

		// Turret update notifier at 100Hz
		// Notifier runs on its own thread — we acquire odometryLock before reading
		// drive pose/speeds so we don't race with the 250Hz odometry thread.
		turretUpdateNotifier =
				new Notifier(
						() -> {
							// Guard drive reads against the odometry thread (same lock Drive uses)
							Drive.odometryLock.lock();
							try {
								m_robotContainer.getSuperstructure().updateTurretTarget();
								m_robotContainer.getSuperstructure().updateTurretAngle();
								m_robotContainer.getSuperstructure().updateShooterVelocity();
							} finally {
								Drive.odometryLock.unlock();
							}
						});

		turretUpdateNotifier.setName("TurretUpdate100Hz");
		turretUpdateNotifier.startPeriodic(0.008333); // 0.01 = 100Hz 0.00833333333333

		try {
			Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
			watchdogField.setAccessible(true);
			Watchdog watchdog = (Watchdog) watchdogField.get(this);
			watchdog.setTimeout(loopOverrunWarningTimeout);
		} catch (Exception e) {
			DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
		}
	}

	/**
	 * Robot periodic method called every 20ms (by default) in all modes.
	 *
	 * <p>Runs the WPILib CommandScheduler, which executes all scheduled commands, updates triggers,
	 * and polls controller inputs. This is the heart of the command-based robot.
	 */
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	/** Called when the robot enters disabled mode. */
	@Override
	public void disabledInit() {}

	/** Called periodically while the robot is disabled. */
	@Override
	public void disabledPeriodic() {}

	/** Called when the robot exits disabled mode. */
	@Override
	public void disabledExit() {}

	@Override
	public void simulationPeriodic() {
		if (RobotConstants.ENABLE_SIM_MANAGER) {
			SimulationManager.getInstance().update();
		}
	}

	/**
	 * Called when autonomous mode begins.
	 *
	 * <p>Retrieves the autonomous command from the robot container and schedules it. The command will
	 * run until it completes or is interrupted by teleop initialization.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
	}

	/** Called periodically during autonomous mode. */
	@Override
	public void autonomousPeriodic() {}

	/** Called when autonomous mode ends. */
	@Override
	public void autonomousExit() {}

	/**
	 * Called when teleop mode begins.
	 *
	 * <p>Cancels the autonomous command if it's still running, allowing teleop commands to take over.
	 */
	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		// m_robotContainer.getSuperstructure().setRobotState(RobotState.IDLE);
	}

	/** Called periodically during teleop mode. */
	@Override
	public void teleopPeriodic() {}

	/** Called when teleop mode ends. */
	@Override
	public void teleopExit() {}

	/**
	 * Called when test mode begins.
	 *
	 * <p>Cancels all commands to provide a clean slate for testing individual subsystems.
	 */
	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	/** Called periodically during test mode. */
	@Override
	public void testPeriodic() {}

	/** Called when test mode ends. */
	@Override
	public void testExit() {}
}
