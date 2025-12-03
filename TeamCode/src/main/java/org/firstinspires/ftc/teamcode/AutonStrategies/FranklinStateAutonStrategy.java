package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class FranklinStateAutonStrategy {

    /**
     * Ok, so like... one problem here is that we need a way to know if a State is "Completed"
     * meaning it's the END of the process.
     * It is ... inelegant... but allowing the "execute" method to return `null` will
     * solve that problem
     */

    public static IAutonStrategy ShootAtRed(FranklinRobot robot, Telemetry telemetry, LinearOpMode opMode) {
        return () ->
        {
            IState currentState = goForwardFor(1500,robot);

            // We really need the opMode here so we don't break any rules~
            while(opMode.opModeIsActive() && currentState != null)
            {
                currentState = currentState.execute();
                opMode.idle();  // it is also important to release some
                // resources back to the robot controller.
            }
        };

    }

    public static IState goForwardFor(long duration, FranklinRobot robot) {
        return() -> {
            robot.getAutonomousRobot().driveTrain.driveForward();
            ThreadExtensions.TrySleep(duration);
            return searchForTag(robot, 24);
        };
    }
    public static IState searchForTag(FranklinRobot robot, int targetTagId) {
        return() -> {
        AprilTagDetection targetTag = findTagById(robot, targetTagId);

        if (targetTag != null) {
            robot.getAutonomousRobot().driveTrain.turnRight();
            ThreadExtensions.TrySleep(175);
            robot.getAutonomousRobot().driveTrain.stop();
            return driveToTag(robot, targetTagId); // Found the target tag!
        }
        robot.getAutonomousRobot().driveTrain.turnRight();
        ThreadExtensions.TrySleep(100);
        return searchForTag(robot, 24);
        };
    }
    public static IState driveToTag(FranklinRobot robot, int tagId) {
        AprilTagDetection targetTag = findTagById(robot, tagId);

        if (targetTag.ftcPose != null) {
            double range = targetTag.ftcPose.range; // Distance to tag (inches)
            double bearing = targetTag.ftcPose.bearing; // Angle to tag (degrees)
            final double TARGET_DISTANCE = 36.0;
            final double BEARING_TOLERANCE = 60.0; // degrees
            final double DISTANCE_TOLERANCE = 10.0; // inches

            if (Math.abs(range - TARGET_DISTANCE) > DISTANCE_TOLERANCE) {
                if (range > TARGET_DISTANCE) {
                    robot.getAutonomousRobot().driveTrain.driveForward();
                    ThreadExtensions.TrySleep(100);
                    robot.getAutonomousRobot().driveTrain.stop();
                    ThreadExtensions.TrySleep(50);
                } else {
                    robot.getAutonomousRobot().driveTrain.driveBackward();
                    ThreadExtensions.TrySleep(100);
                    robot.getAutonomousRobot().driveTrain.stop();
                    ThreadExtensions.TrySleep(50);
                }
            }
            else {
                // We're at the right distance - final alignment
                return shoot(robot);
            }
        }
            ThreadExtensions.TrySleep(100);

        robot.getAutonomousRobot().driveTrain.stop();
        ThreadExtensions.TrySleep(100);
        return driveToTag(robot,tagId);
    }
    public static IState shoot(FranklinRobot robot) {

        return done(robot);
    }
    public static IState done(FranklinRobot robot) {

        return done(robot);
    }
    private static AprilTagDetection findTagById(FranklinRobot robot, int targetTagId) {
        for (AprilTagDetection detection : robot.aprilTagProcessor.getDetections()) {
            if (detection.id == targetTagId) {
                return detection;
            }
        }
        return null; // Target tag not found
    }
}

