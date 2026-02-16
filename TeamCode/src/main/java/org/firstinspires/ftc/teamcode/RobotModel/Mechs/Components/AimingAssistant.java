/* RobotModel/Mechs/Components/AimingAssistant.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Calculates auto-aiming values based on Limelight AprilTag detection
 * Returns turn power, angle adjustment, and aimed status
 */
public class AimingAssistant extends MechComponent {

    public enum AimingMode {
        MANUAL,           // No auto-aim
        HORIZONTAL_ONLY,  // Auto-turn only
        FULL_AUTO         // Auto-turn + angle adjustment
    }

    public class AutonomousAimingAssistant extends AutonomousComponentBehaviors {
        /**
         * Get turn power for autonomous driving toward target
         */
        public double getAutonTurnPower() {
            if (!limelightData.hasAprilTagTarget()) {
                return 0.0;
            }

            double tx = limelightData.getTargetBearing();
            if (Math.abs(tx) < HORIZONTAL_TOLERANCE) {
                return 0.0;
            }

            return tx > 0 ? TURN_POWER : -TURN_POWER;
        }

        /**
         * Get shooter angle adjustment for autonomous
         */
        public double getAutonShooterAngleAdjustment() {
            if (!limelightData.hasAprilTagTarget()) {
                return 0.0;
            }

            double ty = limelightData.getTargetElevation();
            return ty * ANGLE_ADJUSTMENT_RATE;
        }

        /**
         * Check if properly aimed for autonomous
         */
        public boolean isAutonAimed() {
            if (!limelightData.hasAprilTagTarget()) {
                return false;
            }

            double tx = limelightData.getTargetBearing();
            double ty = limelightData.getTargetElevation();

            return Math.abs(tx) < HORIZONTAL_TOLERANCE &&
                    Math.abs(ty) < VERTICAL_TOLERANCE;
        }
    }

    public interface AimingControlStrategy extends IControlStrategy {
        void updateAiming(Gamepad gamepad);
    }

    private AimingMode currentMode = AimingMode.MANUAL;
    private boolean aimingActive = false;
    private final LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightData;
    private final AutonomousAimingAssistant auton = new AutonomousAimingAssistant();

    // Tuning parameters
    private static final double HORIZONTAL_TOLERANCE = 2.0;  // degrees
    private static final double VERTICAL_TOLERANCE = 3.0;    // degrees
    private static final double TURN_POWER = 0.3;
    private static final double ANGLE_ADJUSTMENT_RATE = 0.02; // per degree offset

    /**
     * Initialize AimingAssistant with Limelight data source
     */
    public AimingAssistant(LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightData) {
        super(new AimingControlStrategy() {
            @Override
            public void updateAiming(Gamepad gamepad) {
                // No-op
            }
        });
        this.limelightData = limelightData;
    }

    @Override
    public AutonomousAimingAssistant getAutonomousBehaviors() {
        return auton;
    }

    /**
     * Update aiming mode based on gamepad input
     */
    public void updateAimingMode(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            aimingActive = !aimingActive;
        }

        if (gamepad.dpad_up) {
            currentMode = AimingMode.FULL_AUTO;
        } else if (gamepad.dpad_right) {
            currentMode = AimingMode.HORIZONTAL_ONLY;
        } else if (gamepad.dpad_down) {
            currentMode = AimingMode.MANUAL;
        }
    }

    /**
     * Get turn power for TeleOp
     */
    public double getTurnPower() {
        if (!aimingActive || currentMode == AimingMode.MANUAL) {
            return 0.0;
        }

        if (!limelightData.hasAprilTagTarget()) {
            return 0.0;
        }

        double tx = limelightData.getTargetBearing();

        if (Math.abs(tx) < HORIZONTAL_TOLERANCE) {
            return 0.0;
        }

        return tx > 0 ? TURN_POWER : -TURN_POWER;
    }

    /**
     * Get shooter angle adjustment for TeleOp
     */
    public double getShooterAngleAdjustment() {
        if (currentMode != AimingMode.FULL_AUTO || !aimingActive) {
            return 0.0;
        }

        if (!limelightData.hasAprilTagTarget()) {
            return 0.0;
        }

        double ty = limelightData.getTargetElevation();
        return ty * ANGLE_ADJUSTMENT_RATE;
    }

    /**
     * Check if robot is properly aimed
     */
    public boolean isAimed() {
        if (!limelightData.hasAprilTagTarget()) {
            return false;
        }

        double tx = limelightData.getTargetBearing();
        double ty = limelightData.getTargetElevation();

        return Math.abs(tx) < HORIZONTAL_TOLERANCE &&
                Math.abs(ty) < VERTICAL_TOLERANCE;
    }

    // Getters
    public AimingMode getCurrentMode() {
        return currentMode;
    }

    public boolean isAimingActive() {
        return aimingActive;
    }

    public void setAimingActive(boolean active) {
        this.aimingActive = active;
    }

    @Override
    public void move(Gamepad gamepad) {
        updateAimingMode(gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addLine("=== Aiming Assistant ===");
        telemetry.addData("Mode", currentMode);
        telemetry.addData("Active", aimingActive ? "YES" : "NO");

        if (limelightData.hasAprilTagTarget()) {
            telemetry.addData("Target ID", limelightData.getAprilTagId());
            telemetry.addData("Aimed", isAimed() ? "✓ YES" : "✗ NO");
            telemetry.addData("Horizontal Error", String.format("%.2f°", limelightData.getTargetBearing()));
            telemetry.addData("Vertical Error", String.format("%.2f°", limelightData.getTargetElevation()));
            telemetry.addData("Distance", String.format("%.2f in", limelightData.getDistanceToTarget()));
            telemetry.addData("Turn Power", String.format("%.2f", getTurnPower()));
            telemetry.addData("Angle Adjustment", String.format("%.2f", getShooterAngleAdjustment()));
        } else {
            telemetry.addData("Target", "✗ Not in view");
        }
    }
}
