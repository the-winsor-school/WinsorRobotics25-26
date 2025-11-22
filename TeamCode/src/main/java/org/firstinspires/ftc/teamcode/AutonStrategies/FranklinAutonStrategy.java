package org.firstinspires.ftc.teamcode.AutonStrategies;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class FranklinAutonStrategy {
    // Strategy that targets a specific AprilTag ID

    /**
     * TODO: Comment the methods so we know what they do!
     * @param robot
     * @param franklinRobot
     * @param targetTagId
     * @param telemetry
     * @return
     */
    public static IAutonStrategy TargetSpecificAprilTag(FranklinRobot.AutonomousFranklinRobot robot,
                                                        FranklinRobot franklinRobot, int targetTagId,
                                                        Telemetry telemetry) {
        return () -> {
            // TODO:  This is where the "State Machine" would be /implemented/
            //Get Franklin's front wheels to line up with colorful double marker
            // Phase 1: Search for the specific AprilTag
            if (searchForSpecificAprilTag(robot, franklinRobot, targetTagId, telemetry)) {
                // Phase 2: Navigate to the found AprilTag
                navigateToSpecificAprilTag(robot, franklinRobot, targetTagId, telemetry);
                // Phase 3: Position and shoot
                positionAndShoot(robot, telemetry);
            } else {
                // Fallback behavior if tag not found
                //performFallbackBehavior(robot);
            }
        };
    }


    /**
     * TODO: Comment the methods so we know what they do!
     Helper method to find a specific AprilTag by ID
      */
    private static AprilTagDetection findTagById(FranklinRobot franklinRobot, int targetTagId, Telemetry telemetry) {
        for (AprilTagDetection detection : franklinRobot.aprilTagProcessor.getDetections()) {
            if (detection.id == targetTagId) {
                return detection;
            }
        }
        return null; // Target tag not found
    }

    /**
     * TODO: Comment the methods so we know what they do!
     * @param robot
     * @param franklinRobot
     * @param targetTagId
     * @param telemetry
     * @return
     */
    private static boolean searchForSpecificAprilTag(FranklinRobot.AutonomousFranklinRobot robot, FranklinRobot franklinRobot, int targetTagId, Telemetry telemetry) {
        int searchTime = 0;
        final int MAX_SEARCH_TIME = 4000000; // 4 seconds max search
        robot.driveTrain.driveForward();
        ThreadExtensions.TrySleep(2500); //Adjust to get wheels to hash
        while (searchTime < MAX_SEARCH_TIME) {
            AprilTagDetection targetTag = findTagById(franklinRobot, targetTagId, telemetry);

            if (targetTag != null) {
                robot.driveTrain.turnRight();
                ThreadExtensions.TrySleep(175);
                robot.driveTrain.stop();
                return true; // Found the target tag!
            }

            // Continue searching - rotate slowly
            robot.driveTrain.turnRight();
            ThreadExtensions.TrySleep(100); // Turn for only 100ms (was 150ms)
            robot.driveTrain.stop();
            ThreadExtensions.TrySleep(200); // Pause for 200ms between turns

            searchTime += 300;
        }
        robot.driveTrain.turnRight();
        ThreadExtensions.TrySleep(1000); // Turn for only 100ms (was 150ms)
        robot.driveTrain.stop();
        return false; // Target tag not found
    }

    /**
     * TODO: Comment the methods so we know what they do!
     * @param robot
     * @param franklinRobot
     * @param targetTagId
     * @param telemetry
     */
    private static void navigateToSpecificAprilTag(FranklinRobot.AutonomousFranklinRobot robot, FranklinRobot franklinRobot, int targetTagId, Telemetry telemetry) {
        boolean targetReached = false;
        int navigationTime = 0;
        final int MAX_NAVIGATION_TIME = 6000000; // 6 seconds max navigation
        // TODO:  This While Loop needs some love
        //        First off, the MAX_NAVIGATION_TIME is never used, so it's just an infinite loop.
        while (!targetReached && navigationTime < MAX_NAVIGATION_TIME) {
            AprilTagDetection targetTag = findTagById(franklinRobot, targetTagId, telemetry);

            // TODO:  Look at the "State Machine" lesson.  This crazy
            //        bunch of if statements can be cleaned up a lot~
            //        "State Machine" will also mean you don't need the While Loop~
            if (targetTag != null) {
                // Check if we have pose data (tag must be in library with known size)
                // TODO:  "Invert" this If Statement to reduce a layer of code nesting >_<
                //        this means: If `targetTag.ftcPose` IS null,
                //        then you need to go do a different thing.
                if (targetTag.ftcPose != null) {
                    double range = targetTag.ftcPose.range; // Distance to tag (inches)
                    double bearing = targetTag.ftcPose.bearing; // Angle to tag (degrees)
                    telemetry.addData("range",range);
                    telemetry.addData("bearing",bearing);
                    telemetry.update();
                    // Define target distance (e.g., 18 inches from tag)
                    // TODO:  Make these variables static at the class level
                    //        We don't need to re-declare them every time we are using them ;)
                    final double TARGET_DISTANCE = 36.0;
                    final double BEARING_TOLERANCE = 60.0; // degrees
                    final double DISTANCE_TOLERANCE = 10.0; // inches

                    if (Math.abs(range - TARGET_DISTANCE) > DISTANCE_TOLERANCE) {
                        // Need to adjust distance
//                        if (Math.abs(bearing) > BEARING_TOLERANCE) {
//                            // Turn toward the tag first
//                            TurnTowardsBearing(robot, bearing);
//                            bearing = targetTag.ftcPose.bearing;
//                        }
//                        else {
                            // Move forward or backward to reach target distance
                            if (range > TARGET_DISTANCE) {
                                robot.driveTrain.driveForward();
                                ThreadExtensions.TrySleep(100);
                                robot.driveTrain.stop();
                                ThreadExtensions.TrySleep(50);
                            } else {
                                robot.driveTrain.driveBackward();
                                ThreadExtensions.TrySleep(100);
                                robot.driveTrain.stop();
                                ThreadExtensions.TrySleep(50);
                            }
//                        }
                    } else {
                        // We're at the right distance - final alignment
                        if (Math.abs(bearing) > 3.0) { // Fine-tune alignment
                            if (bearing > 3.0) {
                                robot.driveTrain.turnLeft();
                                ThreadExtensions.TrySleep(50);
                            } else {
                                robot.driveTrain.turnRight();
                                ThreadExtensions.TrySleep(50);
                            }
                        }
                            //else {
//                            // Perfect position reached!
//                            robot.driveTrain.stop();
//                            targetReached = true;
//                        }
                    }
                    // TODO:  Get rid of the empty Else clauses~  unless they shouldn't be empty?
                    //        P.S. "State Machine" will make these go away ;)
                } else {
                    // No pose data - use basic centering
                    //centerTagInView(robot, targetTag, telemetry);
                }
            }
            else {
                // Lost the target tag - stop and search briefly
//                robot.driveTrain.stop();
//                ThreadExtensions.TrySleep(30);
//
//                // Quick search
//                robot.driveTrain.turnLeft();
//                ThreadExtensions.TrySleep(20);
            }
//
            ThreadExtensions.TrySleep(100);
//            navigationTime += 100;
        }

        robot.driveTrain.stop();
        ThreadExtensions.TrySleep(100);
        telemetry.addData("ready to shoot",navigationTime);
    }

    /**
     * TODO: Comment the methods so we know what they do!
     * @param robot
     * @param bearing
     */
    private static void TurnTowardsBearing(FranklinRobot.AutonomousFranklinRobot robot, double bearing) {
        if (bearing > 0) {
            robot.driveTrain.turnLeft();
            ThreadExtensions.TrySleep(10);
        } else {
            robot.driveTrain.turnRight();
            ThreadExtensions.TrySleep(10);

        }
    }

    /** TODO: Comment the methods so we know what they do!
     *
     * @param robot
     * @param detection
     * @param telemetry
     */
    private static void centerTagInView(FranklinRobot.AutonomousFranklinRobot robot, AprilTagDetection detection, Telemetry telemetry) {
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
        }
//        } else {
//            // Tag is centered, move closer
            //robot.driveTrain.driveForward();
            ThreadExtensions.TrySleep(200);
//        }
    }

    /**
     * TODO: Comment the methods so we know what they do!
     * @param robot
     * @param telemetry
     */
    private static void performFallbackBehavior(FranklinRobot.AutonomousFranklinRobot robot, Telemetry telemetry) {
        // What to do if the target tag isn't found
        // Example: drive forward for a set time and shoot anyway
        robot.driveTrain.driveForward();
        ThreadExtensions.TrySleep(2000);
        robot.driveTrain.stop();

        // Still attempt to shoot
        positionAndShoot(robot,telemetry);
    }

    /**
     * TODO: Comment the methods so we know what they do!
     * @param robot
     * @param telemetry
     */
    private static void positionAndShoot(FranklinRobot.AutonomousFranklinRobot robot, Telemetry telemetry) {
        // Position the shooter mechanism
        robot.mechAssembly.FlappyServo.FlappyPos();
        ThreadExtensions.TrySleep(500);

        // Start shooter
        robot.mechAssembly.AutonShooter.StartShoot();
        ThreadExtensions.TrySleep(1000);

        // Drop balls into shooter
        robot.mechAssembly.FlappyServo.FlappyNeg();
        ThreadExtensions.TrySleep(2000);

        // Stop shooter
        robot.mechAssembly.AutonShooter.StopShoot();

        // Reset flappy servo
        robot.mechAssembly.FlappyServo.FlappyPos();
    }
}