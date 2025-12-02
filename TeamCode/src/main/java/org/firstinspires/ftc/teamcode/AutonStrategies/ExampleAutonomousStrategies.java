package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

public class ExampleAutonomousStrategies
{
    /** make it dance! */
    public static IAutonStrategy MecanumAutonDance(Wildbots2025.AutonomousMecanumRobot robot)
    {
        return () ->
        {
            robot.driveTrain.drive(0, 1, 0);
            robot.mechAssembly.drawbridge.goForward();
            ThreadExtensions.TrySleep(500);
            robot.driveTrain.drive(0.5, 0, 0.5);
            ThreadExtensions.TrySleep(500);
            robot.mechAssembly.drawbridge.stop();
            robot.mechAssembly.cascade.goForward();
            ThreadExtensions.TrySleep(1000);
            robot.driveTrain.drive(0, 0, 0);
            robot.mechAssembly.cascade.stop();

            //robot.driveTrain.spinInPlace();
            ThreadExtensions.TrySleep(30000);
        };
    }

    /**
     * Let's write an example StateMachine driven Autonomous Program.
     *
     * Here it is important to pass in all of the things that you could possibly want to
     * use in this AutonStrategy.  You'll pass them along to each of the States as well so
     * that each state can pass those along to the next states.
     *
     * @param robot Franklin!
     * @param telemetry Yes, we should use telemetry~
     * @param opMode we need to pass in the opMode here so we aren't doing anything evil~
     *               We should probably do this in all AutonStrategies
     * @return The Auton Strategy!
     */
    public static IAutonStrategy FranklinDanceStateMachineEdition(FranklinRobot robot, Telemetry telemetry, LinearOpMode opMode) {
        return () ->
        {
            IState currentState = lookForPurple(robot, telemetry);

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
     * starting state:
     * Turn right in a circle until Franklin sees a Purple Blob
     *
     * Keep repeating this until purple is found.
     *
     * once purple if found, move to "driveToPurple" state.
     *
     * @param robot
     * @param telemetry
     * @return either lookForPurple or driveToPurple
     */
    public static IState lookForPurple(FranklinRobot robot, Telemetry telemetry) {
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
                robot.getAutonomousRobot().driveTrain.turnRight();
                ThreadExtensions.TrySleep(50);
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
    public static IState driveTowardPurple(FranklinRobot robot, Telemetry telemetry) {
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
            robot.getAutonomousRobot().driveTrain.driveForward();
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
    public static IState lookForGreen(FranklinRobot robot, Telemetry telemetry) {
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
                robot.getAutonomousRobot().driveTrain.turnLeft();
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
    public static IState driveTowardGreen(FranklinRobot robot, Telemetry telemetry) {
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
            robot.getAutonomousRobot().driveTrain.driveForward();
            ThreadExtensions.TrySleep(50);

            return driveTowardGreen(robot, telemetry);
        };
    }

    public static IState driveInACircleCW(FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("Current State: WOOOOO");
            telemetry.update();

            robot.getAutonomousRobot().driveTrain.turnRight();
            ThreadExtensions.TrySleep(5000);
            return lookForGreen(robot, telemetry);
        };
    }
    public static IState driveInACircleCCW(FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("Current State: WOOOOO");
            telemetry.update();

            robot.getAutonomousRobot().driveTrain.turnLeft();
            ThreadExtensions.TrySleep(5000);
            return null;
        };
    }
}
