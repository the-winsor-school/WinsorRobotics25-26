/* AutonStrategies/SodaAutoAimStrategy.java */
package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.SodapopRobot;

public class SodaAutoAimStrategy {

    /**
     * Auto-aim at AprilTag using Limelight
     * Rotates turret and adjusts hood to aim at target
     */
    public static IAutonStrategy AutoAimAndShoot(SodapopRobot robot, Telemetry telemetry, LinearOpMode opMode) {
        return () -> {
            telemetry.clear();
            telemetry.addLine("=== SODA AUTO-AIM AUTONOMOUS ===");
            telemetry.update();

            IState currentState = searchForTarget(robot, telemetry);

            while (opMode.opModeIsActive() && currentState != null) {
                currentState = currentState.execute();
                opMode.idle();
            }

            telemetry.clear();
            telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
            telemetry.update();
        };
    }

    /**
     * STATE 1: Search for AprilTag
     */
    public static IState searchForTarget(SodapopRobot robot, Telemetry telemetry) {
        return () -> {
            telemetry.clear();
            telemetry.addLine("STATE: Searching for AprilTag");

            if (robot.getAutonomousRobot().mechAssembly.sodaLimelight.hasAprilTagTarget()) {
                telemetry.addLine("✓ Target Found!");
                telemetry.update();
                ThreadExtensions.TrySleep(500);
                return aimTurret(robot, telemetry);
            }

            // Search by rotating turret
            robot.getAutonomousRobot().mechAssembly.sodaTurretTurner.spinFastClockwise();
            telemetry.addData("Status", "Rotating turret...");
            telemetry.update();
            ThreadExtensions.TrySleep(100);

            return searchForTarget(robot, telemetry);
        };
    }

    /**
     * STATE 2: Aim Turret at target (horizontal alignment)
     */
    public static IState aimTurret(SodapopRobot robot, Telemetry telemetry) {
        return () -> {
            telemetry.clear();
            telemetry.addLine("STATE: Aiming Turret");

            double targetX = robot.getAutonomousRobot().mechAssembly.sodaLimelight.getTargetX();
            telemetry.addData("Target X (degrees)", String.format("%.2f", targetX));

            // If target is centered (within 2 degrees), move to hood adjustment
            if (Math.abs(targetX) < 2.0) {
                telemetry.addLine("✓ Turret Aligned!");
                telemetry.update();
                ThreadExtensions.TrySleep(300);
                robot.getAutonomousRobot().mechAssembly.sodaTurretTurner.stop();
                return adjustHood(robot, telemetry);
            }

            // Rotate turret towards target
            if (targetX > 0) {
                // Target is to the right
                robot.getAutonomousRobot().mechAssembly.sodaTurretTurner.rotateClockwise(0.5);
            } else {
                // Target is to the left
                robot.getAutonomousRobot().mechAssembly.sodaTurretTurner.rotateCounterClockwise(0.5);
            }

            telemetry.addData("Action", "Rotating turret...");
            telemetry.update();
            ThreadExtensions.TrySleep(100);

            return aimTurret(robot, telemetry);
        };
    }

    /**
     * STATE 3: Adjust Hood angle based on distance
     */
    public static IState adjustHood(SodapopRobot robot, Telemetry telemetry) {
        return () -> {
            telemetry.clear();
            telemetry.addLine("STATE: Adjusting Hood");

            double targetY = robot.getAutonomousRobot().mechAssembly.sodaLimelight.getTargetY();
            double targetArea = robot.getAutonomousRobot().mechAssembly.sodaLimelight.getTargetArea();

            telemetry.addData("Target Y (degrees)", String.format("%.2f", targetY));
            telemetry.addData("Target Area (%)", String.format("%.2f", targetArea));

            // Simple hood angle calculation based on target Y
            // targetY ranges from -20.5 to 20.5
            // We want to map this to hood angle 0-90 degrees
            double hoodAngle = 45.0 - (targetY * 2.0);  // Adjust multiplier as needed
            hoodAngle = Math.max(0, Math.min(90, hoodAngle));  // Clamp to 0-90

            telemetry.addData("Calculated Hood Angle", String.format("%.1f°", hoodAngle));

            robot.getAutonomousRobot().mechAssembly.sodaHoodAdjuster.setHoodAngle(hoodAngle);
            telemetry.addLine("✓ Hood Adjusted!");
            telemetry.update();
            ThreadExtensions.TrySleep(500);

            return spinUpFlywheel(robot, telemetry);
        };
    }

    /**
     * STATE 4: Spin up flywheel to shooting speed
     */
    public static IState spinUpFlywheel(SodapopRobot robot, Telemetry telemetry) {
        return () -> {
            telemetry.clear();
            telemetry.addLine("STATE: Spinning Up Flywheel");

            robot.getAutonomousRobot().mechAssembly.sodaFlywheel.shootingSpeed();

            double rpm = robot.getAutonomousRobot().mechAssembly.sodaFlywheel.getRPM();
            telemetry.addData("Current RPM", String.format("%.0f", rpm));
            telemetry.addData("Target RPM", "4500");

            // Wait for flywheel to reach speed (4000+ RPM)
            if (rpm > 4000) {
                telemetry.addLine("✓ Flywheel Ready!");
                telemetry.update();
                ThreadExtensions.TrySleep(300);
                return shootArtifacts(robot, telemetry);
            }

            telemetry.addData("Status", "Spinning up...");
            telemetry.update();
            ThreadExtensions.TrySleep(100);

            return spinUpFlywheel(robot, telemetry);
        };
    }

    /**
     * STATE 5: Shoot artifacts (1 or 3)
     */
    public static IState shootArtifacts(SodapopRobot robot, Telemetry telemetry) {
        return () -> {
            telemetry.clear();
            telemetry.addLine("STATE: Shooting Artifacts");

            // Shoot 1 artifact
            telemetry.addLine("Shooting 1 artifact...");
            telemetry.update();
            robot.getAutonomousRobot().mechAssembly.sodaSpindexer.advanceOnePosition();

            ThreadExtensions.TrySleep(1000);

            // Check if we want to shoot more
            // For now, just shoot 1 and stop
            telemetry.addLine("✓ Shot Complete!");
            telemetry.update();
            ThreadExtensions.TrySleep(500);

            robot.getAutonomousRobot().mechAssembly.sodaFlywheel.stop();

            return null;  // End state machine
        };
    }
}
