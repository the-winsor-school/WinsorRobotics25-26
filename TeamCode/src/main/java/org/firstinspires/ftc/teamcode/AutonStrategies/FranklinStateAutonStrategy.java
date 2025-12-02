package org.firstinspires.ftc.teamcode.AutonStrategies;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public interface IState {

        /**
         * we don't actually need the parameter here.
         * @return
         */
        IState execute(/*Robot robot*/);
    }

    public static IAutonStrategy ShootAtRed(FranklinRobot robot, Telemetry telemetry) {
        return ()-> {
            // The problem here is currently we aren't using the "Lambda Expression" form.
            // Therefore, we need ot modify the way we defined all of these things.
            IState currentState = goFowardFor(1500,robot);
        };

    }

    public IState goForwardFor(long duration, FranklinRobot robot) {
        robot.getAutonomousRobot().driveTrain.driveForward();
        ThreadExtensions.TrySleep(duration);
        return searchForTag(robot, 24);
    }
    public IState searchForTag(FranklinRobot robot, int targetTagId) {
        AprilTagDetection targetTag = findTagById(robot, targetTagId);

        if (targetTag != null) {
            robot.getAutonomousRobot().driveTrain.turnRight();
            ThreadExtensions.TrySleep(175);
            robot.getAutonomousRobot().driveTrain.stop();
            return driveToTag(robot,targetTagId); // Found the target tag!
        }
        robot.getAutonomousRobot().driveTrain.turnRight();
        ThreadExtensions.TrySleep(100);
        return searchForTag(robot, 24);
    }
    public IState driveToTag(FranklinRobot robot, int tagId) {
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
    public IState shoot(FranklinRobot robot) {

        return done(robot);
    }
    public IState done(FranklinRobot robot) {

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

