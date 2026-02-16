/* RobotModel/Mechs/Components/AimingAssistant.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.TurnDirection;

/**
 * Handles auto-aiming logic for the shooter
 * Works with Limelight 3A AprilTag detection
 */
public class AimingAssistant {

    public enum AimingMode {
        MANUAL,           // No auto-aim
        HORIZONTAL_ONLY,  // Auto-turn only
        FULL_AUTO         // Auto-turn + angle adjustment
    }

    private AimingMode currentMode = AimingMode.MANUAL;
    private boolean aimingActive = false;

    // Tuning parameters
    private static final double HORIZONTAL_TOLERANCE = 2.0;  // degrees
    private static final double VERTICAL_TOLERANCE = 3.0;    // degrees
    private static final double TURN_POWER = 0.3;
    private static final double ANGLE_ADJUSTMENT_RATE = 0.02; // per degree offset

    public AimingAssistant() {
    }

    /**
     * Enable/disable aiming based on gamepad input
     */
    public void updateAimingMode(Gamepad gamepad) {
        // Right bumper = toggle aiming
        if (gamepad.right_bumper) {
            aimingActive = !aimingActive;
        }

        // D-pad to select aiming mode
        if (gamepad.dpad_up) {
            currentMode = AimingMode.FULL_AUTO;
        } else if (gamepad.dpad_right) {
            currentMode = AimingMode.HORIZONTAL_ONLY;
        } else if (gamepad.dpad_down) {
            currentMode = AimingMode.MANUAL;
        }
    }

    /**
     * Auto-aim during TeleOp
     * Returns motor powers for the drive train
     */
    public double getTurnPower(LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightData) {
        if (!aimingActive || currentMode == AimingMode.MANUAL) {
            return 0.0;
        }

        if (!limelightData.hasAprilTagTarget()) {
            return 0.0;
        }

        double tx = limelightData.getTargetX(); // Horizontal offset

        // If target is centered, no turn needed
        if (Math.abs(tx) < HORIZONTAL_TOLERANCE) {
            return 0.0;
        }

        // Turn toward target
        if (tx > 0) {
            return TURN_POWER; // Turn right
        } else {
            return -TURN_POWER; // Turn left
        }
    }

    /**
     * Get shooter angle adjustment for full auto mode
     */
    public double getShooterAngleAdjustment(LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightData) {
        if (currentMode != AimingMode.FULL_AUTO || !aimingActive) {
            return 0.0;
        }

        if (!limelightData.hasAprilTagTarget()) {
            return 0.0;
        }

        double ty = limelightData.getTargetY(); // Vertical offset

        // Adjust shooter angle based on vertical offset
        return ty * ANGLE_ADJUSTMENT_RATE;
    }

    /**
     * Check if robot is properly aimed
     */
    public boolean isAimed(LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightData) {
        if (!limelightData.hasAprilTagTarget()) {
            return false;
        }

        double tx = limelightData.getTargetX();
        double ty = limelightData.getTargetY();

        return Math.abs(tx) < HORIZONTAL_TOLERANCE &&
                Math.abs(ty) < VERTICAL_TOLERANCE;
    }

    public AimingMode getCurrentMode() {
        return currentMode;
    }

    public boolean isAimingActive() {
        return aimingActive;
    }

    public void setAimingActive(boolean active) {
        this.aimingActive = active;
    }

    public void updateTelemetry(Telemetry telemetry, LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightData) {
        telemetry.addLine("=== Aiming Assistant ===");
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Active", aimingActive ? "YES" : "NO");

        if (limelightData.hasAprilTagTarget()) {
            telemetry.addData("Aimed", isAimed(limelightData) ? "✓ YES" : "✗ NO");
            telemetry.addData("Horizontal Error", String.format("%.2f°", limelightData.getTargetX()));
            telemetry.addData("Vertical Error", String.format("%.2f°", limelightData.getTargetY()));
        } else {
            telemetry.addData("Target", "Not in view");
        }
    }
}
