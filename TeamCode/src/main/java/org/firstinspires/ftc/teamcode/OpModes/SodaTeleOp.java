/* OpModes/SodaTeleOp.java */
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.SodapopMA;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.AimingAssistant;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightVision;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.SodapopRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;

@TeleOp(name = "Soda TeleOp")
public class SodaTeleOp extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SodapopRobot(hardwareMap);

        telemetry.addLine("=== Sodapop TeleOp ===");
        telemetry.addLine("Left Stick = Drive");
        telemetry.addLine("Right Stick = Turn");
        telemetry.addLine();
        telemetry.addLine("=== Auto-Aim Controls ===");
        telemetry.addLine("Right Bumper = Toggle Auto-Aim");
        telemetry.addLine("D-Pad Up = Full Auto");
        telemetry.addLine("D-Pad Right = Horizontal Only");
        telemetry.addLine("D-Pad Down = Manual");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Standard robot update
            robot.update(gamepad1, gamepad2);

            // Get aiming data for telemetry (without 'var')
            SodapopRobot sodapopRobot = (SodapopRobot) robot;
            SodapopMA sodapopMA = sodapopRobot.getMechAssembly();
            AimingAssistant aimingAssistant = sodapopMA.getAimingAssistant();
            LimelightVision.AutonomousLimelightVision limelightBehaviors = sodapopMA.getLimelightBehaviors();

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
                telemetry.addData("Bearing (tx)", String.format("%.2f°", limelightBehaviors.getTargetX()));
                telemetry.addData("Elevation (ty)", String.format("%.2f°", limelightBehaviors.getTargetY()));
                telemetry.addData("Distance", String.format("%.2f in", limelightBehaviors.getDistanceToTarget()));
                telemetry.addData("Aimed", aimingAssistant.isAimed(limelightBehaviors) ? "✓ YES" : "✗ NO");
            } else {
                telemetry.addData("Target Found", "✗ NO");
                telemetry.addData("Status", "Searching for AprilTag...");
            }

            telemetry.update();
        }
    }
}
