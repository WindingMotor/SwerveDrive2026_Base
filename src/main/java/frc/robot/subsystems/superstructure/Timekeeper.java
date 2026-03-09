// Copyright (c) 2025 - 2026 : FRC 2106 : The Junkyard Dogs
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class Timekeeper {

    double countDown = 0.0;
    boolean isHubActive = false;

    public Timekeeper() {}

    // Called in superstructure periodic loop
    public void update() {

        isHubActive = isHubActive();
        countDown = computeCountdown();

        SmartDashboard.putNumber("countDown", countDown);
        SmartDashboard.putBoolean("isHubActive", isHubActive);
    }

    // Returns seconds until the hub next changes state.
    // When active  → countdown to when hub goes inactive.
    // When inactive → countdown to when hub goes active.
    // Returns 0.0 when no further state change will occur.
    private double computeCountdown() {
        // Only meaningful during teleop.
        if (!DriverStation.isTeleopEnabled()) {
            return 0.0;
        }

        double matchTime = DriverStation.getMatchTime();
        boolean shift1Active = computeShift1Active();
        boolean currentState = isHubActiveAtMatchTime(matchTime, shift1Active);

        // Walk through each threshold in descending order.
        // At each threshold the hub state may or may not flip — skip ones that don't.
        double[] thresholds = {130.0, 105.0, 80.0, 55.0, 30.0};
        for (double threshold : thresholds) {
            if (matchTime > threshold) {
                // Probe the state just inside the next zone (0.5 s below threshold).
                boolean stateAfter = isHubActiveAtMatchTime(threshold - 0.5, shift1Active);
                if (stateAfter != currentState) {
                    // This threshold is the next real state flip.
                    return matchTime - threshold;
                }
                // No flip here — the always-active zone borders absorb this crossing.
            }
        }

        return 0.0; // End-game or no remaining change.
    }

    // Resolves whether shift 1 is active for our alliance.
    // Returns false (and is safe to call) if alliance or game data are unavailable.
    private boolean computeShift1Active() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) {
            return true; // Default: assume hub is active.
        }
        boolean redInactiveFirst = switch (gameData.charAt(0)) {
            case 'R' -> true;
            case 'B' -> false;
            default -> { yield false; } // Safe default.
        };
        return switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };
    }

    // Pure function: hub state at an arbitrary match time given a resolved shift1Active.
    private boolean isHubActiveAtMatchTime(double matchTime, boolean shift1Active) {
        if (matchTime > 130) return true;
        if (matchTime > 105) return shift1Active;
        if (matchTime > 80)  return !shift1Active;
        if (matchTime > 55)  return shift1Active;
        if (matchTime > 30)  return !shift1Active;
        return true; // End-game.
    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return true;

        boolean shift1Active = computeShift1Active();
        return isHubActiveAtMatchTime(matchTime, shift1Active);
    }
}