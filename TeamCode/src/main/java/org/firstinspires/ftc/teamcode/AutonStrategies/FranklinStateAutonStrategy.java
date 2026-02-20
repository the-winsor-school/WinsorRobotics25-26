//package org.firstinspires.ftc.teamcode.AutonStrategies;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Extensions.IState;
//import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
//import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//public class FranklinStateAutonStrategy {
//
//    public static IAutonStrategy ShootAtRed(FranklinRobot robot, Telemetry telemetry, LinearOpMode opMode) {
//        return () ->
//        {
//            telemetry.clear();
//            telemetry.addLine("=== FRANKLIN AUTONOMOUS STARTED ===");
//            telemetry.addData("Target", "Red AprilTag ID 24 For Far");
//            telemetry.addData("Strategy", "Drive Forward -> Search -> Approach -> Shoot");
//            telemetry.update();
//
//            IState currentState = goForwardFor(4000, robot, telemetry);
//
//            while(opMode.opModeIsActive() && currentState != null)
//            {
//                currentState = currentState.execute();
//                opMode.idle();
//            }
//
//            telemetry.clear();
//            telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
//            telemetry.update();
//        };
//    }
//
//    public static IState goForwardFor(long duration, FranklinRobot robot, Telemetry telemetry) {
//        return () ->
//        {
//            telemetry.clear();
//            telemetry.addLine("STATE: Initial Forward Drive");
//            telemetry.addData("Duration", duration + " ms");
//            telemetry.addData("Action", "Driving forward to search position");
//            telemetry.update();
//
//            robot.getAutonomousRobot().driveTrain.driveForward();
//            ThreadExtensions.TrySleep(duration);
//            robot.getAutonomousRobot().driveTrain.stop();
//
//            telemetry.addLine("Forward drive complete - starting tag search");
//            telemetry.update();
//            return searchForTag(robot, 24, telemetry);
//        };
//    }
//
//    public static IState goBackwardFor(long duration, FranklinRobot robot, Telemetry telemetry) {
//        return () ->
//        {
//            telemetry.clear();
//            telemetry.addLine("STATE: Final Backward Drive");
//            telemetry.addData("Duration", duration + " ms");
//            telemetry.addData("Action", "Driving backward to position");
//            telemetry.update();
//
//            robot.getAutonomousRobot().driveTrain.driveBackward();
//            ThreadExtensions.TrySleep(duration);
//            robot.getAutonomousRobot().driveTrain.stop();
//
//            telemetry.addLine("Backward drive complete - ending auton");
//            telemetry.update();
//            return null;
//        };
//    }
//
//    public static IState searchForTag(FranklinRobot robot, int targetTagId, Telemetry telemetry) {
//        return () -> {
//            telemetry.clear();
//            telemetry.addLine("STATE: Searching for AprilTag");
//            telemetry.addData("Target Tag ID", targetTagId);
//
//            robot.getAutonomousRobot().driveTrain.stop();
//            ThreadExtensions.TrySleep(200);
//            AprilTagDetection targetTag = findTagById(robot, targetTagId);
//
//            int totalTags = robot.aprilTagProcessor.getDetections().size();
//            telemetry.addData("Total Tags Detected", totalTags);
//
//            if (totalTags > 0) {
//                telemetry.addLine("Detected Tag IDs:");
//                for (AprilTagDetection detection : robot.aprilTagProcessor.getDetections()) {
//                    telemetry.addData("  Tag", "ID " + detection.id);
//                }
//            }
//
//            if (targetTag != null)
//            {
//                telemetry.addLine("✓ TARGET TAG FOUND!");
//                telemetry.addData("Tag ID", targetTag.id);
//                if (targetTag.ftcPose != null) {
//                    telemetry.addData("Range", String.format("%.1f inches", targetTag.ftcPose.range));
//                    telemetry.addData("Bearing", String.format("%.1f degrees", targetTag.ftcPose.bearing));
//                }
//                telemetry.addData("Next Action", "Fine positioning then approach");
//                telemetry.update();
//
//                ///  THIS IS THE BUG
//            /*
//                robot.getAutonomousRobot().driveTrain.turnLeft();
//                ThreadExtensions.TrySleep(375);
//
//             */
//
//                robot.getAutonomousRobot().driveTrain.stop();
//                return driveToTag(robot, targetTagId, telemetry);
//            }
//
//            telemetry.addData("Status", "Tag not found - continuing search");
//            telemetry.addData("Action", "Turning right to search");
//            telemetry.update();
//
//            robot.getAutonomousRobot().driveTrain.turnRight();
//            ThreadExtensions.TrySleep(100);
//            robot.getAutonomousRobot().driveTrain.stop();
//            ThreadExtensions.TrySleep(50);
//
//            return searchForTag(robot, 24, telemetry);
//        };
//    }
//
//    // **FIXED: Added return statement**
//    public static IState driveToTag(FranklinRobot robot, int tagId, Telemetry telemetry) {
//        return () -> {  // **This return was missing!**
//            telemetry.clear();
//            telemetry.addLine("STATE: Approaching Target");
//
//            robot.getAutonomousRobot().driveTrain.stop();
//            ThreadExtensions.TrySleep(100);
//            AprilTagDetection targetTag = findTagById(robot, tagId);
//
//            if (targetTag == null) {
//                telemetry.addLine("❌ Lost target tag!");
//                telemetry.addData("Action", "Returning to search");
//                telemetry.update();
//                return searchForTag(robot, tagId, telemetry);
//            }
//
//            telemetry.addData("Target Tag ID", targetTag.id);
//
//            if (targetTag.ftcPose != null)
//            {
//                double range = targetTag.ftcPose.range;
//                double bearing = targetTag.ftcPose.bearing;
//                final double TARGET_DISTANCE = 35.0;
//                final double DISTANCE_TOLERANCE = 10.0;
//
//                telemetry.addData("Current Range", String.format("%.1f inches", range));
//                telemetry.addData("Target Range", TARGET_DISTANCE + " inches");
//                telemetry.addData("Range Error", String.format("%.1f inches", Math.abs(range - TARGET_DISTANCE)));
//                telemetry.addData("Bearing", String.format("%.1f degrees", bearing));
//
//                if (Math.abs(range - TARGET_DISTANCE) > DISTANCE_TOLERANCE)
//                {
//                    if (range > TARGET_DISTANCE)
//                    {
//                        telemetry.addData("Action", "Moving FORWARD (too far)");
//                        telemetry.update();
//
//                        robot.getAutonomousRobot().driveTrain.driveForward();
//                        ThreadExtensions.TrySleep(100);
//                        robot.getAutonomousRobot().driveTrain.stop();
//                        ThreadExtensions.TrySleep(10);
//                    }
//                    else
//                    {
//                        telemetry.addData("Action", "Moving BACKWARD (too close)");
//                        telemetry.update();
//
//                        robot.getAutonomousRobot().driveTrain.driveBackward();
//                        ThreadExtensions.TrySleep(100);
//                        robot.getAutonomousRobot().driveTrain.stop();
//                        ThreadExtensions.TrySleep(50);
//                    }
//                    return driveToTag(robot, tagId, telemetry);
//                }
//                else
//                {
//                    telemetry.addLine("✓ OPTIMAL DISTANCE REACHED!");
//                    telemetry.addData("Final Range", String.format("%.1f inches", range));
//                    telemetry.addData("Next State", "SHOOTING SEQUENCE");
//                    telemetry.update();
//                    robot.getAutonomousRobot().driveTrain.turnRight();
//                    ThreadExtensions.TrySleep(100);
//
//                    robot.getAutonomousRobot().driveTrain.stop();
//                    return shoot(robot, telemetry);
//                }
//            }
//            else
//            {
//                telemetry.addLine("⚠ No pose data available");
//                telemetry.addData("Action", "Proceeding to shoot anyway");
//                telemetry.update();
//                robot.getAutonomousRobot().driveTrain.turnRight();
//                ThreadExtensions.TrySleep(100);
//                return shoot(robot, telemetry);
//            }
//        };
//    }
//
//    // **FIXED: Corrected method names and autonomous access**
//    public static IState shoot(FranklinRobot robot, Telemetry telemetry)
//    {
//        return () -> {
//            telemetry.clear();
//            telemetry.addLine("STATE: SHOOTING SEQUENCE");
//            telemetry.addLine("======================");
//
//
//            telemetry.addData("Step 1", "Starting shooter motor...");
//            telemetry.update();
//            robot.getAutonomousRobot().mechAssembly.AutonShooter.SetSpeed(-0.8);
//            ThreadExtensions.TrySleep(1000);
//
//            telemetry.addData("Step 2", "Releasing balls - FlappyServo DOWN...");
//            telemetry.update();
//            robot.getAutonomousRobot().mechAssembly.FlappyServo.FlappyNeg();  // **Was FlappyNeg()**
//            ThreadExtensions.TrySleep(2000);
//
//            telemetry.addData("Step 3", "Opening Churro Intake...");
//            telemetry.update();
//
//            robot.getAutonomousRobot().mechAssembly.NewIntakeServo.newIntakeUp();  // **needs to be churro**
//
//            ThreadExtensions.TrySleep(2000);
//
//            robot.getAutonomousRobot().mechAssembly.NewIntakeServo.newIntakeStop();
//            ThreadExtensions.TrySleep(2000);
//
//            telemetry.addData("Step 4", "Stopping shooter...");
//            telemetry.update();
//            robot.getAutonomousRobot().mechAssembly.AutonShooter.StopShoot();
//            ThreadExtensions.TrySleep(1000);
//
//            telemetry.clear();
//            telemetry.addLine("🎯 SHOOTING COMPLETE! 🎯");
//            telemetry.update();
//            robot.getAutonomousRobot().driveTrain.turnRight();
//            ThreadExtensions.TrySleep(1000);
//            robot.getAutonomousRobot().driveTrain.driveForward();
//            ThreadExtensions.TrySleep(1000);
//            return goBackwardFor(20, robot, telemetry);  // End state machine
//        };
//    }
//    public static IAutonStrategy ShootAtRedFromGoal(FranklinRobot robot, Telemetry telemetry, LinearOpMode opMode) {
//        return () ->
//        {
//            telemetry.clear();
//            telemetry.addLine("=== FRANKLIN AUTONOMOUS STARTED ===");
//            telemetry.addData("Target From Goal", "Red AprilTag ID 24");
//            telemetry.addData("Strategy", "Drive Forward -> Turn 180 -> Shoot -> Exit Launch Zone");
//            telemetry.update();
//
//            IState currentState = goForwardsForFar(1500, robot, telemetry);

//            while(opMode.opModeIsActive() && currentState != null)
//            {
//                currentState = currentState.execute();
//                opMode.idle();
//            }
//
//            telemetry.clear();
//            telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
//            telemetry.update();
//        };
//    }
//    public static IState goForwardsForFar(long duration, FranklinRobot robot, Telemetry telemetry) {
//        return () ->
//        {
//            telemetry.clear();
//            telemetry.addLine("STATE: Initial Forward Drive");
//            telemetry.addData("Duration", duration + " ms");
//            telemetry.addData("Action", "Driving forward to search position");
//            telemetry.update();
//
//            robot.getAutonomousRobot().driveTrain.driveForward();
//            ThreadExtensions.TrySleep(duration);
//            robot.getAutonomousRobot().driveTrain.stop();
//
//            telemetry.addLine("Forward drive complete - turning");
//            telemetry.update();
//            return turnAround(/*figure out duration*/1200, robot, telemetry);
//        };
//    }
//    public static IState turnAround(long duration, FranklinRobot robot, Telemetry telemetry) {
//        return () ->
//        {
//            telemetry.clear();
//            telemetry.addLine("STATE: Initial Forward Drive");
//            telemetry.addData("Duration", duration + " ms");
//            telemetry.addData("Action", "Turning right to shoot");
//            telemetry.update();
//
//            robot.getAutonomousRobot().driveTrain.turnRight();
//            ThreadExtensions.TrySleep(duration);
//            robot.getAutonomousRobot().driveTrain.stop();
//
//            telemetry.addLine("Turning right to shoot complete - starting shooter");
//            telemetry.update();
//            return shooter(robot, telemetry);
//        };
//    }
//    public static IState shooter(FranklinRobot robot, Telemetry telemetry)
//    {
//        return () -> {
//            telemetry.clear();
//            telemetry.addLine("STATE: SHOOTING SEQUENCE");
//            telemetry.addLine("======================");
//
//
//            telemetry.addData("Step 1", "Starting shooter motor...");
//            telemetry.update();
//            robot.getAutonomousRobot().mechAssembly.AutonShooter.SetSpeed(-0.8);
//            ThreadExtensions.TrySleep(1000);
//
//            telemetry.addData("Step 2", "Releasing balls - FlappyServo DOWN...");
//            telemetry.update();
//            robot.getAutonomousRobot().mechAssembly.FlappyServo.FlappyNeg();  // **Was FlappyNeg()**
//            ThreadExtensions.TrySleep(2000);
//
//            telemetry.addData("Step 3", "Opening Churro Intake...");
//            telemetry.update();
//
//            robot.getAutonomousRobot().mechAssembly.NewIntakeServo.newIntakeUp();  // **needs to be churro**
//
//            ThreadExtensions.TrySleep(2000);
//
//            robot.getAutonomousRobot().mechAssembly.NewIntakeServo.newIntakeStop();
//            ThreadExtensions.TrySleep(2000);
//
//            telemetry.addData("Step 4", "Stopping shooter...");
//            telemetry.update();
//            robot.getAutonomousRobot().mechAssembly.AutonShooter.StopShoot();
//            ThreadExtensions.TrySleep(1000);
//
//            telemetry.clear();
//            telemetry.addLine("🎯 SHOOTING COMPLETE! 🎯");
//            telemetry.update();
//            robot.getAutonomousRobot().driveTrain.turnRight();
//            ThreadExtensions.TrySleep(/*tweak this and below*/ 1000);
//            robot.getAutonomousRobot().driveTrain.driveForward();
//            ThreadExtensions.TrySleep(1000);
//            return goBackwardFor(20, robot, telemetry);  // End state machine
//        };
//    }
//    private static AprilTagDetection findTagById(FranklinRobot robot, int targetTagId) {
//        for (AprilTagDetection detection : robot.aprilTagProcessor.getDetections()) {
//            if (detection.id == targetTagId) {
//                return detection;
//            }
//        }
//        return null;
//    }
//}
package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class FranklinStateAutonStrategy {

    public static IAutonStrategy ShootAtRed(FranklinRobot robot, Telemetry telemetry, LinearOpMode opMode) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("=== FRANKLIN AUTONOMOUS STARTED ===");
            telemetry.addData("Target", "Red AprilTag ID 24 For Far");
            telemetry.addData("Strategy", "Drive Forward -> Search -> Approach -> Shoot");
            telemetry.update();

            IState currentState = goForwardFor(4000, robot, telemetry);

            while(opMode.opModeIsActive() && currentState != null)
            {
                currentState = currentState.execute();
                opMode.idle();
            }

            telemetry.clear();
            telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
            telemetry.update();
        };
    }

    public static IState goForwardFor(long duration, FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("STATE: Initial Forward Drive");
            telemetry.addData("Duration", duration + " ms");
            telemetry.addData("Action", "Driving forward to search position");
            telemetry.update();

            robot.getAutonomousRobot().driveTrain.driveForward();
            ThreadExtensions.TrySleep(duration);
            robot.getAutonomousRobot().driveTrain.stop();

            telemetry.addLine("Forward drive complete - starting tag search");
            telemetry.update();
            return searchForTag(robot, 24, telemetry);
        };
    }

    public static IState goBackwardFor(long duration, FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("STATE: Final Backward Drive");
            telemetry.addData("Duration", duration + " ms");
            telemetry.addData("Action", "Driving backward to position");
            telemetry.update();

            robot.getAutonomousRobot().driveTrain.driveBackward();
            ThreadExtensions.TrySleep(duration);
            robot.getAutonomousRobot().driveTrain.stop();

            telemetry.addLine("Backward drive complete - ending auton");
            telemetry.update();
            return null;
        };
    }

    public static IState searchForTag(FranklinRobot robot, int targetTagId, Telemetry telemetry) {
        return () -> {
            telemetry.clear();
            telemetry.addLine("STATE: Searching for AprilTag");
            telemetry.addData("Target Tag ID", targetTagId);

            robot.getAutonomousRobot().driveTrain.stop();
            ThreadExtensions.TrySleep(200);
            AprilTagDetection targetTag = findTagById(robot, targetTagId);

            int totalTags = robot.aprilTagProcessor.getDetections().size();
            telemetry.addData("Total Tags Detected", totalTags);

            if (totalTags > 0) {
                telemetry.addLine("Detected Tag IDs:");
                for (AprilTagDetection detection : robot.aprilTagProcessor.getDetections()) {
                    telemetry.addData("  Tag", "ID " + detection.id);
                }
            }

            if (targetTag != null)
            {
                telemetry.addLine("✓ TARGET TAG FOUND!");
                telemetry.addData("Tag ID", targetTag.id);
                if (targetTag.ftcPose != null) {
                    telemetry.addData("Range", String.format("%.1f inches", targetTag.ftcPose.range));
                    telemetry.addData("Bearing", String.format("%.1f degrees", targetTag.ftcPose.bearing));
                }
                telemetry.addData("Next Action", "Fine positioning then approach");
                telemetry.update();

                ///  THIS IS THE BUG
            /*
                robot.getAutonomousRobot().driveTrain.turnLeft();
                ThreadExtensions.TrySleep(375);

             */

                robot.getAutonomousRobot().driveTrain.stop();
                return driveToTag(robot, targetTagId, telemetry);
            }

            telemetry.addData("Status", "Tag not found - continuing search");
            telemetry.addData("Action", "Turning left to search");
            telemetry.update();

            robot.getAutonomousRobot().driveTrain.turnRight();
            ThreadExtensions.TrySleep(100);
            robot.getAutonomousRobot().driveTrain.stop();
            ThreadExtensions.TrySleep(50);

            return searchForTag(robot, 20, telemetry);
        };
    }

    // **FIXED: Added return statement**
    public static IState driveToTag(FranklinRobot robot, int tagId, Telemetry telemetry) {
        return () -> {  // **This return was missing!**
            telemetry.clear();
            telemetry.addLine("STATE: Approaching Target");

            robot.getAutonomousRobot().driveTrain.stop();
            ThreadExtensions.TrySleep(100);
            AprilTagDetection targetTag = findTagById(robot, tagId);

            if (targetTag == null) {
                telemetry.addLine("❌ Lost target tag!");
                telemetry.addData("Action", "Returning to search");
                telemetry.update();
                return searchForTag(robot, tagId, telemetry);
            }

            telemetry.addData("Target Tag ID", targetTag.id);

            if (targetTag.ftcPose != null)
            {
                double range = targetTag.ftcPose.range;
                double bearing = targetTag.ftcPose.bearing;
                final double TARGET_DISTANCE = 35.0;
                final double DISTANCE_TOLERANCE = 10.0;

                telemetry.addData("Current Range", String.format("%.1f inches", range));
                telemetry.addData("Target Range", TARGET_DISTANCE + " inches");
                telemetry.addData("Range Error", String.format("%.1f inches", Math.abs(range - TARGET_DISTANCE)));
                telemetry.addData("Bearing", String.format("%.1f degrees", bearing));

                if (Math.abs(range - TARGET_DISTANCE) > DISTANCE_TOLERANCE)
                {
                    if (range > TARGET_DISTANCE)
                    {
                        telemetry.addData("Action", "Moving FORWARD (too far)");
                        telemetry.update();

                        robot.getAutonomousRobot().driveTrain.driveForward();
                        ThreadExtensions.TrySleep(100);
                        robot.getAutonomousRobot().driveTrain.stop();
                        ThreadExtensions.TrySleep(10);
                    }
                    else
                    {
                        telemetry.addData("Action", "Moving BACKWARD (too close)");
                        telemetry.update();

                        robot.getAutonomousRobot().driveTrain.driveBackward();
                        ThreadExtensions.TrySleep(100);
                        robot.getAutonomousRobot().driveTrain.stop();
                        ThreadExtensions.TrySleep(50);
                    }
                    return driveToTag(robot, tagId, telemetry);
                }
                else
                {
                    telemetry.addLine("✓ OPTIMAL DISTANCE REACHED!");
                    telemetry.addData("Final Range", String.format("%.1f inches", range));
                    telemetry.addData("Next State", "SHOOTING SEQUENCE");
                    telemetry.update();
                    robot.getAutonomousRobot().driveTrain.turnLeft();
                    ThreadExtensions.TrySleep(100);

                    robot.getAutonomousRobot().driveTrain.stop();
                    robot.getAutonomousRobot().driveTrain.turnLeft();
                    ThreadExtensions.TrySleep(100);
                    return shoot(robot, telemetry);
                }
            }
            else
            {
                telemetry.addLine("⚠ No pose data available");
                telemetry.addData("Action", "Proceeding to shoot anyway");
                telemetry.update();
                robot.getAutonomousRobot().driveTrain.turnLeft();
                ThreadExtensions.TrySleep(100);
                return shoot(robot, telemetry);
            }
        };
    }

    // **FIXED: Corrected method names and autonomous access**
    public static IState shoot(FranklinRobot robot, Telemetry telemetry)
    {
        return () -> {
            telemetry.clear();
            telemetry.addLine("STATE: SHOOTING SEQUENCE");
            telemetry.addLine("======================");

            telemetry.addData("Step 1", "Starting shooter motor...");
            telemetry.update();
            robot.getAutonomousRobot().mechAssembly.AutonShooter.SetSpeed(-0.8);
            ThreadExtensions.TrySleep(1000);

            telemetry.addData("Step 2", "Releasing balls - FlappyServo DOWN...");
            telemetry.update();
            robot.getAutonomousRobot().mechAssembly.FlappyServo.FlappyNeg();  // **Was FlappyNeg()**
            ThreadExtensions.TrySleep(2000);

            telemetry.addData("Step 3", "Opening Churro Intake...");
            telemetry.update();

            robot.getAutonomousRobot().mechAssembly.NewIntakeServo.newIntakeUp();  // **needs to be churro**
            ThreadExtensions.TrySleep(2000);

            robot.getAutonomousRobot().mechAssembly.NewIntakeServo.newIntakeStop();
            robot.getAutonomousRobot().mechAssembly.AutonShooter.SetSpeed(0.9);
            ThreadExtensions.TrySleep(4000);

            telemetry.addData("Step 4", "Stopping shooter...");
            telemetry.update();
            robot.getAutonomousRobot().mechAssembly.AutonShooter.StopShoot();
            ThreadExtensions.TrySleep(1000);

            telemetry.clear();
            telemetry.addLine("🎯 SHOOTING COMPLETE! 🎯");
            telemetry.update();
            robot.getAutonomousRobot().driveTrain.turnLeft();
            ThreadExtensions.TrySleep(1000);
            robot.getAutonomousRobot().driveTrain.driveForward();
            ThreadExtensions.TrySleep(1000);
            return goBackwardFor(20, robot, telemetry);  // End state machine
        };
    }
    public static IAutonStrategy ShootAtRedFromGoal(FranklinRobot robot, Telemetry telemetry, LinearOpMode opMode) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("=== FRANKLIN AUTONOMOUS STARTED ===");
            telemetry.addData("Target From Goal", "Red AprilTag ID 24");
            telemetry.addData("Strategy", "Drive Forward -> Turn 180 -> Shoot -> Exit Launch Zone");
            telemetry.update();

            IState currentState = goForwardsForFar(2200, robot, telemetry);

            while(opMode.opModeIsActive() && currentState != null)
            {
                currentState = currentState.execute();
                opMode.idle();
            }

            telemetry.clear();
            telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
            telemetry.update();
        };
    }
    public static IState goForwardsForFar(long duration, FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("STATE: Initial Forward Drive");
            telemetry.addData("Duration", duration + " ms");
            telemetry.addData("Action", "Driving forward to search position");
            telemetry.update();

            robot.getAutonomousRobot().driveTrain.driveForward();
            ThreadExtensions.TrySleep(duration);
            robot.getAutonomousRobot().driveTrain.stop();

            telemetry.addLine("Forward drive complete - turning");
            telemetry.update();
            return turnAround(/*figure out duration*/1250, robot, telemetry);
        };
    }
    public static IState turnAround(long duration, FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("STATE: Initial Forward Drive");
            telemetry.addData("Duration", duration + " ms");
            telemetry.addData("Action", "Turning left to shoot");
            telemetry.update();

            robot.getAutonomousRobot().driveTrain.turnRight();
            ThreadExtensions.TrySleep(duration);
            robot.getAutonomousRobot().driveTrain.stop();

            telemetry.addLine("Turning right to shoot complete - starting shooter");
            telemetry.update();
            return shooter(robot, telemetry);
        };
    }
    public static IState shooter(FranklinRobot robot, Telemetry telemetry)
    {
        return () -> {
            telemetry.clear();
            telemetry.addLine("STATE: SHOOTING SEQUENCE");
            telemetry.addLine("======================");
            robot.getAutonomousRobot().driveTrain.driveForward();
            ThreadExtensions.TrySleep(100);
            robot.getAutonomousRobot().driveTrain.turnRight();
            ThreadExtensions.TrySleep(100);
            robot.getAutonomousRobot().driveTrain.stop();

            telemetry.addData("Step 1", "Starting shooter motor...");
            telemetry.update();
            robot.getAutonomousRobot().mechAssembly.AutonShooter.SetSpeed(-0.72);
            ThreadExtensions.TrySleep(1000);

            telemetry.addData("Step 2", "Releasing balls - FlappyServo DOWN...");
            telemetry.update();
            robot.getAutonomousRobot().mechAssembly.FlappyServo.FlappyNeg();  // **Was FlappyNeg()**
            ThreadExtensions.TrySleep(2000);

            telemetry.addData("Step 3", "Opening Churro Intake...");
            telemetry.update();

            robot.getAutonomousRobot().mechAssembly.NewIntakeServo.newIntakeUp();  // **needs to be churro**

            ThreadExtensions.TrySleep(2000);

            robot.getAutonomousRobot().mechAssembly.NewIntakeServo.newIntakeStop();
            robot.getAutonomousRobot().mechAssembly.AutonShooter.SetSpeed(-0.82);
            ThreadExtensions.TrySleep(4000);

            ThreadExtensions.TrySleep(5000);

            telemetry.addData("Step 4", "Stopping shooter...");
            telemetry.update();
            robot.getAutonomousRobot().mechAssembly.AutonShooter.StopShoot();
            ThreadExtensions.TrySleep(1000);

            telemetry.clear();
            telemetry.addLine("🎯 SHOOTING COMPLETE! 🎯");
            telemetry.update();
            robot.getAutonomousRobot().driveTrain.turnRight();
            ThreadExtensions.TrySleep(/*tweak this and below*/ 1000);
            robot.getAutonomousRobot().driveTrain.driveForward();
            ThreadExtensions.TrySleep(2000);
            return goBackwardFor(20, robot, telemetry);  // End state machine
        };
    }
    private static AprilTagDetection findTagById(FranklinRobot robot, int targetTagId) {
        for (AprilTagDetection detection : robot.aprilTagProcessor.getDetections()) {
            if (detection.id == targetTagId) {
                return detection;
            }
        }
        return null;
    }
}


