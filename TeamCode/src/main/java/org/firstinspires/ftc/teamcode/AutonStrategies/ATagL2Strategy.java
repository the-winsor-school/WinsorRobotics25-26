package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

public class ATagL2Strategy {
    public static IAutonStrategy track(BillyRobot robot, Telemetry telemetry, LinearOpMode opMode)
                 {
        return () ->
        {

            IState currentState = lookForTag(robot, telemetry);

            while (opMode.opModeIsActive() && currentState != null )
            {
                currentState = currentState.execute();
                opMode.idle();
            }

            
            robot.getAutonomousRobot().mechAssembly.autonFlywheel.shoot(0.77);
            ThreadExtensions.TrySleep(3500);
            ThreadExtensions.TrySleep(200);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.retractPusher();
            ThreadExtensions.TrySleep(1500);


            //second ball
            robot.getAutonomousRobot().mechAssembly.autonFlywheel.shoot(0.78);
            ThreadExtensions.TrySleep(1000);
            robot.getAutonomousRobot().mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(200);
            robot.getAutonomousRobot().mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);
            robot.getAutonomousRobot().mechAssembly.autonIntake.reverseIntake();
            ThreadExtensions.TrySleep(100);
            robot.getAutonomousRobot().mechAssembly.autonFlywheel.shoot(0.785);
            robot.getAutonomousRobot().mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);
            ThreadExtensions.TrySleep(200);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.retractPusher();
            ThreadExtensions.TrySleep(1500);

            //third ball
            robot.getAutonomousRobot().mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(1000);
            robot.getAutonomousRobot().mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);
            ThreadExtensions.TrySleep(200);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.retractPusher();
            ThreadExtensions.TrySleep(200);

            robot.getAutonomousRobot().driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(1000);
        };
    }

    public static IState lookForTag(BillyRobot robot, Telemetry telemetry)
    {
        return () ->
        {
            Limelight3A limelight = robot.limelight;

            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid())
            {
                result.getPipelineIndex();

                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);

                return turnToTag(robot, telemetry, tx);
            }
            else
            {
                telemetry.addData("Limelight", "No Targets");

                return turnToTag(robot, telemetry, 0.0);
            }
        };
    }

    public static IState turnToTag(BillyRobot robot, Telemetry telemetry, Double angle)
    {
        return () ->
        {
            if (angle == 0) {
                return lookForTag(robot, telemetry);
            }

            robot.getAutonomousRobot().driveTrain.turnToAngle(angle);

            ThreadExtensions.TrySleep(100);

            return lookForTag(robot, telemetry);
        };
    }


    public static IState turnToAngle(BillyRobot robot, Telemetry telemetry, double targetAngle)
    {
        return () ->
        {
            double currentHeading = robot.getAutonomousRobot().getHeading();

            double error = robot.getAutonomousRobot().angleWrap(targetAngle - currentHeading);

            double kP = 0.01;   //have to test this
            double turnPower = error * kP;

            turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Heading", currentHeading);
            telemetry.addData("Error", error);

            if (Math.abs(error) < 2)
            {
                robot.getAutonomousRobot().driveTrain.drive(0,0,0);
                return null;
            }

            robot.getAutonomousRobot().driveTrain.drive(0, 0, turnPower);

            return turnToAngle(robot, telemetry, targetAngle);
        };
    }

    public static IState StraightDrive(BillyRobot robot,
                                       Telemetry telemetry,
                                       double xPower,
                                       double yPower,
                                       long durationMs)
    {
        long startTime = System.currentTimeMillis();

        double targetHeading = robot.getAutonomousRobot().getHeading();

        return new IState() {
            @Override
            public IState execute()
            {
                double currentHeading = robot.getAutonomousRobot().getHeading();

                double error = robot.getAutonomousRobot().angleWrap(targetHeading - currentHeading);

                double kP = 0.02;   //have to test this
                double correction = error * kP;

                correction = Math.max(-0.3, Math.min(0.3, correction));

                telemetry.addData("Locked Heading", targetHeading);
                telemetry.addData("Current Heading", currentHeading);
                telemetry.addData("Error", error);
                telemetry.addData("Correction", correction);

                robot.getAutonomousRobot().driveTrain.drive(
                        xPower,
                        yPower,
                        correction
                );

                if (System.currentTimeMillis() - startTime > durationMs)
                {
                    robot.getAutonomousRobot().driveTrain.drive(0,0,0);
                    return null;
                }

                return this;
            }
        };
    }
}