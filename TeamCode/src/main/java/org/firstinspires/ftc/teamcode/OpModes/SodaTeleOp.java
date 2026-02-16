/* OpModes/SodaTeleOp.java */
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.SodapopMA;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.AimingAssistant;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightForSoda;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.SodapopRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;

@TeleOp(name = "Soda TeleOp")
public class SodaTeleOp extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SodapopRobot(hardwareMap);

        telemetry.addLine("=== Sodapop TeleOp Driver ===");
        telemetry.addLine("Left Stick = Drive");
        telemetry.addLine("Right Stick = Turn");
        telemetry.addLine("Right Trigger = Inake In");
        telemetry.addLine("Left Trigger = Inake Out");
        telemetry.addLine("=== Sodapop TeleOp Mech ===");
        telemetry.addLine("Right Bumper = Start Shooter");
        telemetry.addLine("Left Bumper = Reverse Shooter");
        telemetry.addLine("Right Trigger = Intake In");
        telemetry.addLine("Right Trigger = Intake Out");
        telemetry.addLine("Left Arrow = Cycle Spindexer 1 left");
        telemetry.addLine("Right Arrow = Cycle Spindexer 1 right");
        telemetry.addLine("Y = Disable Auto-Aim");
        telemetry.addLine("A = Re-Enable Auto-Aim");
        telemetry.addLine("Right Joystick = Turn Turret");
        telemetry.addLine("Left Joystick = Turn Hood");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Standard robot update
            robot.update(gamepad1, gamepad2);

            // Get aiming data for telemetry
            SodapopRobot sodapopRobot = (SodapopRobot) robot;
            SodapopMA sodapopMA = sodapopRobot.getMechAssembly();
            AimingAssistant aimingAssistant = sodapopMA.getAimingAssistant();
            LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightBehaviors =
                    sodapopMA.getLimelightBehaviors();

            // Update telemetry
            robot.updateTelemetry(telemetry);

            // Add aiming telemetry
            telemetry.addLine();
            telemetry.addLine("=== Aiming Status ===");
            telemetry.addData("Mode", aimingAssistant.getCurrentMode());
            telemetry.addData("Active", aimingAssistant.isAimingActive() ? "YES" : "NO");

            if (limelightBehaviors.hasAprilTagTarget()) {
                telemetry.addData("Target Found", "✓ YES");
                telemetry.addData("AprilTag ID", limelightBehaviors.getAprilTagId());
                telemetry.addData("Bearing (tx)", String.format("%.2f°", limelightBehaviors.getTargetBearing()));
                telemetry.addData("Elevation (ty)", String.format("%.2f°", limelightBehaviors.getTargetElevation()));
                telemetry.addData("Distance", String.format("%.2f in", limelightBehaviors.getDistanceToTarget()));
                telemetry.addData("Aimed", aimingAssistant.isAimed() ? "✓ YES" : "✗ NO");  // ✓ Fixed
            } else {
                telemetry.addData("Target Found", "✗ NO");
                telemetry.addData("Status", "Searching for AprilTag...");
            }


            telemetry.update();
        }
    }
}
