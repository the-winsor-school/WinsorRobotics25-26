package org.firstinspires.ftc.teamcode.AutonStrategies;

import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class FranklinAutonStrategy {

    // Strategy that targets a specific AprilTag ID
    public static IAutonStrategy TargetSpecificAprilTag(FranklinRobot.AutonomousFranklinRobot robot, FranklinRobot franklinRobot, int targetTagId) {
        return () -> {
            // Phase 1: Search for the specific AprilTag
            if (searchForSpecificAprilTag(robot, franklinRobot, targetTagId)) {
                // Phase 2: Navigate to the found AprilTag
                navigateToSpecificAprilTag(robot, franklinRobot, targetTagId);

                // Phase 3: Position and shoot
                positionAndShoot(robot);
            } else {
                // Fallback behavior if tag not found
                //performFallbackBehavior(robot);
            }
        };
    }

    // Helper method to find a specific AprilTag by ID
    private static AprilTagDetection findTagById(FranklinRobot franklinRobot, int targetTagId) {
        for (AprilTagDetection detection : franklinRobot.aprilTagProcessor.getDetections()) {
            if (detection.id == targetTagId) {
                return detection;
            }
        }
        return null; // Target tag not found
    }

    private static boolean searchForSpecificAprilTag(FranklinRobot.AutonomousFranklinRobot robot, FranklinRobot franklinRobot, int targetTagId) {
        int searchTime = 0;
        final int MAX_SEARCH_TIME = 4000000; // 4 seconds max search

        while (searchTime < MAX_SEARCH_TIME) {
            AprilTagDetection targetTag = findTagById(franklinRobot, targetTagId);

            if (targetTag != null) {
                robot.driveTrain.stop();
                return true; // Found the target tag!
            }

            // Continue searching - rotate slowly
            robot.driveTrain.turnRight();
            ThreadExtensions.TrySleep(50); // Turn for only 100ms (was 150ms)
            robot.driveTrain.stop();
            ThreadExtensions.TrySleep(25); // Pause for 200ms between turns

            searchTime += 300;
        }

        robot.driveTrain.stop();
        return false; // Target tag not found
    }

    private static void navigateToSpecificAprilTag(FranklinRobot.AutonomousFranklinRobot robot, FranklinRobot franklinRobot, int targetTagId) {
        boolean targetReached = false;
        int navigationTime = 0;
        final int MAX_NAVIGATION_TIME = 6000000; // 6 seconds max navigation

        while (!targetReached && navigationTime < MAX_NAVIGATION_TIME) {
            AprilTagDetection targetTag = findTagById(franklinRobot, targetTagId);

            if (targetTag != null) {
                // Check if we have pose data (tag must be in library with known size)
                if (targetTag.ftcPose != null) {
                    double range = targetTag.ftcPose.range; // Distance to tag (inches)
                    double bearing = targetTag.ftcPose.bearing; // Angle to tag (degrees)

                    // Define target distance (e.g., 18 inches from tag)
                    final double TARGET_DISTANCE = 24.0;
                    final double BEARING_TOLERANCE = 60.0; // degrees
                    final double DISTANCE_TOLERANCE = 2.0; // inches

                    if (Math.abs(range - TARGET_DISTANCE) > DISTANCE_TOLERANCE) {
                        // Need to adjust distance
                        if (Math.abs(bearing) > BEARING_TOLERANCE) {
                            // Turn toward the tag first
                            if (bearing > 0) {
                                robot.driveTrain.turnLeft();
                            } else {
                                robot.driveTrain.turnRight();
                            }
                        } else {
                            // Move forward or backward to reach target distance
                            if (range > TARGET_DISTANCE) {
                                robot.driveTrain.driveForward();
                                ThreadExtensions.TrySleep(50);
                                robot.driveTrain.stop();
                                ThreadExtensions.TrySleep(100);
                            } else {
                                robot.driveTrain.driveBackward();
                                ThreadExtensions.TrySleep(50);
                                robot.driveTrain.stop();
                                ThreadExtensions.TrySleep(100);
                            }
                        }
                    } else {
                        // We're at the right distance - final alignment
                        if (Math.abs(bearing) > 3.0) { // Fine-tune alignment
                            if (bearing > 0) {
                                robot.driveTrain.turnLeft();
                            } else {
                                robot.driveTrain.turnRight();
                            }
                        } else {
                            // Perfect position reached!
                            robot.driveTrain.stop();
                            targetReached = true;
                        }
                    }
                } else {
                    // No pose data - use basic centering
                    centerTagInView(robot, targetTag);
                }
            } else {
                // Lost the target tag - stop and search briefly
                robot.driveTrain.stop();
                ThreadExtensions.TrySleep(300);

                // Quick search
                robot.driveTrain.turnLeft();
                ThreadExtensions.TrySleep(200);
            }

            ThreadExtensions.TrySleep(100);
            navigationTime += 100;
        }

        robot.driveTrain.stop();
    }

    private static void centerTagInView(FranklinRobot.AutonomousFranklinRobot robot, AprilTagDetection detection) {
        double tagCenterX = detection.center.x;
        double imageCenterX = 320; // Assuming 640x480 resolution
        double offset = tagCenterX - imageCenterX;

        if (Math.abs(offset) > 40) {
            if (offset > 0) { // Tag is to the right
                robot.driveTrain.turnRight();
            } else { // Tag is to the left
                robot.driveTrain.turnLeft();
            }
            ThreadExtensions.TrySleep(150);
        } else {
            // Tag is centered, move closer
            robot.driveTrain.driveForward();
            ThreadExtensions.TrySleep(200);
        }
    }

    private static void performFallbackBehavior(FranklinRobot.AutonomousFranklinRobot robot) {
        // What to do if the target tag isn't found
        // Example: drive forward for a set time and shoot anyway
        robot.driveTrain.driveForward();
        ThreadExtensions.TrySleep(2000);
        robot.driveTrain.stop();

        // Still attempt to shoot
        positionAndShoot(robot);
    }

    private static void positionAndShoot(FranklinRobot.AutonomousFranklinRobot robot) {
        // Position the shooter mechanism
        robot.mechAssembly.FlappyServo.FlappyUp();
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