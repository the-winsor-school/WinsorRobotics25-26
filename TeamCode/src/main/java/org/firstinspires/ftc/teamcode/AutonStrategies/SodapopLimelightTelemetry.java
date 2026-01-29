/* AutonStrategies/SodapopLimelightTelemetry.java */
package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.SodapopRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightForSoda;

public class SodapopLimelightTelemetry {

    public static IAutonStrategy DisplayPoseDataForTags20And24(SodapopRobot robot,
                                                               Telemetry telemetry,
                                                               LinearOpMode opMode) {
        return () -> {
            telemetry.clear();
            telemetry.addLine("=== SODAPOP LIMELIGHT POSE TELEMETRY ===");
            telemetry.addLine("Monitoring AprilTags 20 and 24");
            telemetry.addLine("Using Limelight 4 via NetworkTables");
            telemetry.addLine("=====================================");
            telemetry.update();

            while (opMode.opModeIsActive()) {
                LimelightForSoda.AutonomousLimelightForSodaBehaviors limelightBehaviors = robot.getAutonomousRobot().mechAssembly.limelightForSoda;

                telemetry.clear();
                telemetry.addLine("=== LIMELIGHT 4 POSE DATA ===");

                if (limelightBehaviors.hasAprilTagTarget()) {
                    int detectedId = limelightBehaviors.getAprilTagId();

                    if (detectedId == 20 || detectedId == 24) {
                        telemetry.addData("🎯 TARGET TAG FOUND", "ID %d", detectedId);
                        telemetry.addLine("───────────────────────────");

                        // Basic detection data
                        double tx = limelightBehaviors.getTargetX();
                        double ty = limelightBehaviors.getTargetY();
                        double ta = limelightBehaviors.getTargetArea();

                        telemetry.addData("X Offset (tx)", "%.2f°", tx);
                        telemetry.addData("Y Offset (ty)", "%.2f°", ty);
                        telemetry.addData("Target Area (ta)", "%.2f%%", ta);
                        telemetry.addLine("───────────────────────────");

                        // 3D Pose Data
                        double poseX = limelightBehaviors.getRobotPoseX();
                        double poseY = limelightBehaviors.getRobotPoseY();
                        double yaw = limelightBehaviors.getRobotPoseRotation();

                        telemetry.addLine("🤖 ROBOT POSE (botpose):");
                        telemetry.addData("X Position", "%.2f units", poseX);
                        telemetry.addData("Y Position", "%.2f units", poseY);
                        telemetry.addData("Yaw", "%.2f°", yaw);
                        telemetry.addLine("───────────────────────────");

                        // Distance calculation
                        double distance = Math.sqrt(poseX * poseX + poseY * poseY);
                        telemetry.addData("Distance to Tag", "%.2f units", distance);

                    } else {
                        telemetry.addData("❌ WRONG TAG", "Found ID %d (want 20 or 24)", detectedId);
                    }

                } else {
                    telemetry.addLine("🔍 SEARCHING FOR APRILTAGS...");
                    telemetry.addLine("No AprilTag detected by Limelight");
                }

                telemetry.addLine("───────────────────────────");
                telemetry.addLine("Press STOP to end telemetry");
                telemetry.update();

                ThreadExtensions.TrySleep(100);
                opMode.idle();
            }
        };
    }
}
