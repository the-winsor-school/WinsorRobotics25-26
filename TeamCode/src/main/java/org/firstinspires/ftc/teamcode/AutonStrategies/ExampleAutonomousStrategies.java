package org.firstinspires.ftc.teamcode.AutonStrategies;

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
     * @return The Auton Strategy!
     */
    public static IAutonStrategy FranklinDanceStateMachineEdition(FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {
            IState currentState = lookForPurple(robot, telemetry);

            while(currentState != null)
            {
                currentState = currentState.execute();
            }
        };
    }

    public static IState lookForPurple(FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("Current State: looking for purple");
            List<ColorBlobLocatorProcessor.Blob> blobs = robot.purpleBallProcessor.getBlobs();
            if(blobs.isEmpty() || !blobs.stream().anyMatch(b -> b.getBoxFit().center.x > 300 &&))
            {
                // Print the current state message on the DriverStation.
                telemetry.addLine("I don't see Purple yet...");
                telemetry.update();
                robot.getAutonomousRobot().driveTrain.turnRight();
                ThreadExtensions.TrySleep(50);
                return lookForPurple(robot, telemetry);
            }

            

            telemetry.addLine("I don't see Purple yet...");
            telemetry.update();
        };
    }

    public static IState driveTowardPurple(FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {

        };
    }

    public static IState lookForGreen(FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {

        };
    }
    public static IState driveTowardGreen(FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {

        };
    }

    public static IState driveInACircleCW(FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {

        };
    }
    public static IState driveInACircleCCW(FranklinRobot robot, Telemetry telemetry) {
        return () ->
        {

        };
    }
}
