package org.firstinspires.ftc.teamcode.AutonStrategies;

import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class FranklinAutonStrategy {

    public static IAutonStrategy AprilTagNavigationStrategy(FranklinRobot.AutonomousFranklinRobot robot, FranklinRobot franklinRobot) {
        return () -> {
            // Phase 1: Look for AprilTag
            searchForAprilTag(robot, franklinRobot);

            // Phase 2: Navigate to AprilTag
            navigateToAprilTag(robot, franklinRobot);

            // Phase 3: Position and shoot
            positionAndShoot(robot);
        };
    }

    private static void searchForAprilTag(FranklinRobot.AutonomousFranklinRobot robot, FranklinRobot franklinRobot) {
        // Rotate slowly to find AprilTag
        int searchTime = 0;
        boolean tagFound = false;

        while (searchTime < 3000 && !tagFound) { // Search for 3 seconds max
            // Get AprilTag detections from the vision portal
            if (franklinRobot.aprilTagProcessor.getDetections().size() > 0) {
                tagFound = true;
                robot.driveTrain.stop(); // Stop searching
            } else {
                // Rotate to search for tag
                robot.driveTrain.turnRight();
                ThreadExtensions.TrySleep(100);
                searchTime += 100;
            }
        }

        robot.driveTrain.stop();
    }

    private static void navigateToAprilTag(FranklinRobot.AutonomousFranklinRobot robot, FranklinRobot franklinRobot) {
        boolean targetReached = false;
        int navigationTime = 0;

        while (!targetReached && navigationTime < 5000) { // Navigate for max 5 seconds
            if (franklinRobot.aprilTagProcessor.getDetections().size() > 0) {
                AprilTagDetection detection = franklinRobot.aprilTagProcessor.getDetections().get(0);

                // Check if we have pose data (tag must be in library)
                if (detection.ftcPose != null) {
                    double range = detection.ftcPose.range; // Distance to tag
                    double bearing = detection.ftcPose.bearing; // Angle to tag
                    double yaw = detection.ftcPose.yaw; // Tag rotation

                    // Navigate based on AprilTag pose
                    if (range > 24.0) { // If more than 24 inches away
                        if (Math.abs(bearing) > 10) { // Need to turn toward tag
                            if (bearing > 0) {
                                robot.driveTrain.turnLeft();
                            } else {
                                robot.driveTrain.turnRight();
                            }
                        } else { // Drive forward toward tag
                            robot.driveTrain.driveForward();
                        }
                    } else {
                        // We're close enough - stop
                        robot.driveTrain.stop();
                        targetReached = true;
                    }
                } else {
                    // No pose data - just center the tag in view
                    centerTagInView(robot, detection);
                }
            } else {
                // Lost the tag - stop and search again
                robot.driveTrain.stop();
                ThreadExtensions.TrySleep(500);
            }

            ThreadExtensions.TrySleep(100);
            navigationTime += 100;
        }

        robot.driveTrain.stop();
    }

    private static void centerTagInView(FranklinRobot.AutonomousFranklinRobot robot, AprilTagDetection detection) {
        // Use the tag's center position to align robot
        double tagCenterX = detection.center.x;
        double imageCenterX = 320; // Assuming 640x480 resolution

        double offset = tagCenterX - imageCenterX;

        if (Math.abs(offset) > 50) { // If tag is significantly off-center
            if (offset > 0) { // Tag is to the right
                robot.driveTrain.turnRight();
            } else { // Tag is to the left
                robot.driveTrain.turnLeft();
            }
            ThreadExtensions.TrySleep(200);
        } else {
            // Tag is centered, drive forward
            robot.driveTrain.driveForward();
            ThreadExtensions.TrySleep(300);
        }
    }

    private static void positionAndShoot(FranklinRobot.AutonomousFranklinRobot robot) {
        // Position the shooter mechanism
        robot.mechAssembly.FlappyServo.FlappyUp(); // Prepare to drop balls
        ThreadExtensions.TrySleep(500);

        // Start shooter
        robot.mechAssembly.AutonShooter.StartShoot();
        ThreadExtensions.TrySleep(1000);

        // Drop balls into shooter
        robot.mechAssembly.FlappyServo.FlappyDown();
        ThreadExtensions.TrySleep(2000);

        // Stop shooter
        robot.mechAssembly.AutonShooter.StopShoot();

        // Reset flappy servo
        robot.mechAssembly.FlappyServo.FlappyUp();
    }
}
