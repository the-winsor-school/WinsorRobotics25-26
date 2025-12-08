package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.TurnDirection;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.RyanRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

//TODO: SOMEONE PLEASE MAKE A SHOOTER METHOD IDK HOW THE SHOOTER WORKS FOR RYAN ROBOT PLEASE

public class RyanAutonStrategy {
    /*
    the line below i'm not sure about... help
    (the public static one, shouldn't use wildbots2025 but...
    idk what the autonomousmecanumrobot is)
    
    Cox - This looks like a good starter for the Ryan auton.  You have to Robot, Telemetry and the opMode~
    Basically the important thing here is that you need to have access to everything you're possibly going to 
    want to use in the Autonomous action.
     */
    public static IAutonStrategy Green(RyanRobot robot, Telemetry telemetry, LinearOpMode opMode) {
        return () -> {
            IState currentState = lookForGreen(robot, telemetry);

            while (opMode.opModeIsActive() && currentState != null )
            {
                currentState = currentState.execute();
                opMode.idle();
            }
        };
    }

    public static IAutonStrategy Purple(RyanRobot robot, Telemetry telemetry, LinearOpMode opMode) {
        return () -> {
            IState currentState = lookForPurple(robot, telemetry);

            while (opMode.opModeIsActive() && currentState != null )
            {
                currentState = currentState.execute();
                opMode.idle();;
            }
        };
    }

    public static IState lookForPurple(RyanRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("Current State: looking for purple");

            // Get some purple blobs~
            List<ColorBlobLocatorProcessor.Blob> blobs = robot.purpleBallProcessor.getBlobs();

            // if there aren't any, or if they're not centered in the frame...
            if(blobs.isEmpty() || blobs.stream().noneMatch(b -> b.getBoxFit().center.x > 300 && b.getBoxFit().center.x < 380))
            {
                // Print the current state message on the DriverStation.
                telemetry.addLine("I don't see Purple yet...");
                telemetry.update();
                // turn right a bit..
                // TODO: I did give the silliest way possible to do this as an example ;) lol
                robot.getAutonomousRobot().driveTrain.spin(TurnDirection.RIGHT);
                ThreadExtensions.TrySleep(50);
                // TODO:  you might also need to tell the drivetrain to STOP before it tries to do more vision processing.
                return lookForPurple(robot, telemetry);
            }

            telemetry.addLine("THATS PURPLE!!");
            telemetry.update();
            ThreadExtensions.TrySleep(500);
            return driveTowardPurple(robot, telemetry);
        };
    }

    /**
     * Assumes we have FOUND purple
     *
     * if no purple is seen, we lost it -- return to lookForPurple
     *
     * if purple is covering enough area of the camera.. we found it!  return drive in a circle cw
     *
     * otherwise, keep driving forward
     *
     * @param robot
     * @param telemetry
     * @return
     */
    public static IState driveTowardPurple(RyanRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("Current State: GETTIN THAT PURPLE THING");

            // Get some purple blobs~
            List<ColorBlobLocatorProcessor.Blob> blobs = robot.purpleBallProcessor.getBlobs();

            // first way out--Lost the PURPLE thing
            if(blobs.isEmpty())
            {
                telemetry.addLine("I lost it....");
                telemetry.update();
                ThreadExtensions.TrySleep(500);
                return lookForPurple(robot, telemetry);
            }


            // Second path out--The purple thing is big enough to count as I got it~
            if(blobs.stream().anyMatch(b -> b.getContourArea() > 100))
            {
                telemetry.addLine("I GOT IT!");
                telemetry.update();
                ThreadExtensions.TrySleep(500);
                return driveInACircleCW(robot, telemetry);
            }

            // otherwise, keep driving forward.
            telemetry.addLine("OHH LAWD HE COMMIN'");
            telemetry.update();
            robot.getAutonomousRobot().driveTrain.drive(1,1 ,0);
            ThreadExtensions.TrySleep(50);

            return driveTowardPurple(robot, telemetry);
        };
    }


    /**
     * Turn left in a circle until Franklin sees a Green Blob
     *
     * Keep repeating this until green is found.
     *
     * once green is found, move to "driveToGreen" state.
     *
     * @param robot
     * @param telemetry
     * @return either lookForGreen or driveToGreen
     */
    public static IState lookForGreen(RyanRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("Current State: looking for GREEN");

            // Get some green blobs~
            List<ColorBlobLocatorProcessor.Blob> blobs = robot.greenBallProcessor.getBlobs();

            // if there aren't any, or if they're not centered in the frame...
            if(blobs.isEmpty() || blobs.stream().noneMatch(b -> b.getBoxFit().center.x > 300 && b.getBoxFit().center.x < 380))
            {
                // Print the current state message on the DriverStation.
                telemetry.addLine("I don't see GREEN yet...");
                telemetry.update();
                // turn right a bit..
                robot.getAutonomousRobot().driveTrain.spin(TurnDirection.LEFT);
                ThreadExtensions.TrySleep(50);
                return lookForGreen(robot, telemetry);
            }

            telemetry.addLine("THATS GREEN!!");
            telemetry.update();
            ThreadExtensions.TrySleep(500);
            return driveTowardGreen(robot, telemetry);
        };
    }

    /**
     * Assumes we have FOUND green
     *
     * if no green is seen, we lost it -- return to lookForGreen
     *
     * if green is covering enough area of the camera.. we found it!  return drive in a circle ccw
     *
     * otherwise, keep driving forward
     *
     * @param robot
     * @param telemetry
     * @return
     */
    public static IState driveTowardGreen(RyanRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("Current State: GETTIN THAT GREEN THING");

            // Get some green blobs~
            List<ColorBlobLocatorProcessor.Blob> blobs = robot.greenBallProcessor.getBlobs();

            // first way out--Lost the GREEN thing
            if(blobs.isEmpty())
            {
                telemetry.addLine("I lost it....");
                telemetry.update();
                ThreadExtensions.TrySleep(500);
                return lookForGreen(robot, telemetry);
            }


            // Second path out--The green thing is big enough to count as I got it~
            if(blobs.stream().anyMatch(b -> b.getContourArea() > 100))
            {
                telemetry.addLine("I GOT IT!");
                telemetry.update();
                ThreadExtensions.TrySleep(500);
                return driveInACircleCCW(robot, telemetry);
            }

            // otherwise, keep driving forward.
            telemetry.addLine("OHH LAWD HE COMMIN'");
            telemetry.update();
            robot.getAutonomousRobot().driveTrain.drive(1,1, 0);
            ThreadExtensions.TrySleep(50);

            return driveTowardGreen(robot, telemetry);
        };
    }

    public static IState driveInACircleCW(RyanRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("Current State: WOOOOO");
            telemetry.update();

            robot.getAutonomousRobot().driveTrain.spin(TurnDirection.RIGHT);
            ThreadExtensions.TrySleep(5000);
            return lookForGreen(robot, telemetry);
        };
    }
    public static IState driveInACircleCCW(RyanRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("Current State: WOOOOO");
            telemetry.update();

            robot.getAutonomousRobot().driveTrain.spin(TurnDirection.LEFT);
            ThreadExtensions.TrySleep(5000);
            return null;
        };
    }
}
