package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class BACKUPForWhenFranklinRefusesToWork {

    /**
     * Ok, so like... one problem here is that we need a way to know if a State is "Completed"
     * meaning it's the END of the process.
     * It is ... inelegant... but allowing the "execute" method to return `null` will
     * solve that problem
     */

    public static IAutonStrategy ShootAtRed(FranklinRobot robot, Telemetry telemetry, LinearOpMode opMode) {
        return () ->
        {
            IState currentState = goForwardFor(1000,robot, telemetry);

            // We really need the opMode here so we don't break any rules~
            while(opMode.opModeIsActive() && currentState != null)
            {
                currentState = currentState.execute();
                opMode.idle();  // it is also important to release some
                // resources back to the robot controller.
            }
        };

    }

    /**
     * TODO:  Add telemetry to /all/ of these states ;-;  Telemetry is SUPER important for debugging~
     */
    public static IState goForwardFor(long duration, FranklinRobot robot, Telemetry telemetry) {
        return() ->
        {
            // TODO:  The `driveForward()` method here should probably get a "speed" parameter~
            robot.getAutonomousRobot().driveTrain.driveForward();
            ThreadExtensions.TrySleep(duration);
            return(done(robot, telemetry));
            //return searchForTag(robot, 24, telemetry);
        };
    }


    public static IState searchForTag(FranklinRobot robot, int targetTagId, Telemetry telemetry) {
        return() -> {
            AprilTagDetection targetTag = findTagById(robot, targetTagId);

            if (targetTag != null)
            {
                // TODO:  Question:  Why are we continuing to turn right after finding the tag? Franlin likes it better that way
                robot.getAutonomousRobot().driveTrain.turnLeft();
                ThreadExtensions.TrySleep(375);
                robot.getAutonomousRobot().driveTrain.stop();
                return driveToTag(robot, targetTagId, telemetry); // Found the target tag!
            }


            // TODO:  Another thing that might be helpful is to modify the "turnRight()" method on the driveTrain
            //        It would be helpful to be able to give it a desired "speed" so you can slow this turning down~
            robot.getAutonomousRobot().driveTrain.turnLeft(0.3F);
            ThreadExtensions.TrySleep(100);
            robot.getAutonomousRobot().driveTrain.stop();
            ThreadExtensions.TrySleep(50);
            // We need to stop the robot here~  That's why its spinning constantly
            return searchForTag(robot, 24, telemetry);
        };
    }

    /**
     * Don't forget to JavaDoc comment your methods~ >_<
     */
    public static IState driveToTag(FranklinRobot robot, int tagId, Telemetry telemetry) {
        AprilTagDetection targetTag = findTagById(robot, tagId);

        if (targetTag != null && targetTag.ftcPose != null)
        {
            // TODO:  talk with Susan and Eleanor about updating this part~
            double range = targetTag.ftcPose.range; // Distance to tag (inches)
            double bearing = targetTag.ftcPose.bearing; // Angle to tag (degrees)
            final double TARGET_DISTANCE = 36.0;
            final double BEARING_TOLERANCE = 60.0; // degrees
            final double DISTANCE_TOLERANCE = 10.0; // inches

            // TODO:  We've done a lot of untangling code, but these tripply-nested If Statements are still painful ;)
            //        We can talk about how to dis-entangle this by "Inverting the IF statements"
            if (Math.abs(range - TARGET_DISTANCE) > DISTANCE_TOLERANCE)
            {
                if (range > TARGET_DISTANCE)
                {
                    robot.getAutonomousRobot().driveTrain.driveForward();
                    ThreadExtensions.TrySleep(100);
                    robot.getAutonomousRobot().driveTrain.stop();
                    ThreadExtensions.TrySleep(10);
                }
                else
                {
                    robot.getAutonomousRobot().driveTrain.driveBackward();
                    ThreadExtensions.TrySleep(100);
                    robot.getAutonomousRobot().driveTrain.stop();
                    ThreadExtensions.TrySleep(50);
                }
            }
            else
            {
                // We're at the right distance - final alignment
                return shoot(robot, telemetry);
            }
        }
        ThreadExtensions.TrySleep(100);

        robot.getAutonomousRobot().driveTrain.stop();
        ThreadExtensions.TrySleep(100);
        return driveToTag(robot,tagId, telemetry);
    }


    public static IState shoot(FranklinRobot robot, Telemetry telemetry)
    {
        // TODO:  shoot things?
        telemetry.addLine("starting shooting");
        telemetry.update();

        // Position the shooter mechanism
        robot.getAutonomousRobot().mechAssembly.FlappyServo.FlappyPos();

        // Start shooter
        robot.getAutonomousRobot().mechAssembly.AutonShooter.StartShoot();

        // Drop balls into shooter
        robot.getAutonomousRobot().mechAssembly.FlappyServo.FlappyNeg();

        // Stop shooter
        robot.getAutonomousRobot().mechAssembly.AutonShooter.StopShoot();

        // Reset flappy servo
        robot.getAutonomousRobot().mechAssembly.FlappyServo.FlappyPos();




        // Instead of a Done thing, the way to tell that we are finished is to return `null`
        return done(robot, telemetry);
    }

    /**
     * this method does not need to exist~
     */
    public static IState done(FranklinRobot robot, Telemetry telemetry) {

        return done(robot, telemetry);
    }

    /**
     * TODO: This is a great helper method ;) fix this comment to explain what it does~
     */
    private static AprilTagDetection findTagById(FranklinRobot robot, int targetTagId) {
        for (AprilTagDetection detection : robot.aprilTagProcessor.getDetections()) {
            if (detection.id == targetTagId) {
                return detection;
            }
        }
        return null; // Target tag not found
    }
}



