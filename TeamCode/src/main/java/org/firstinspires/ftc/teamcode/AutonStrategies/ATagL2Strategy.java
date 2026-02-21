package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

public class ATagL2Strategy {
    public static IAutonStrategy track(BillyRobot robot, Telemetry telemetry, LinearOpMode opMode, int tagID)
                 {
        return () ->
        {

            IState turretState = lookForTag(robot, telemetry,tagID);
            IState driveState = ShootBalls(robot,telemetry);

            while (opMode.opModeIsActive() && driveState != null)
            {
                turretState = turretState.execute();
                driveState = driveState.execute();
                opMode.idle();
            }


        };
    }

    public static IState lookForTag(BillyRobot robot, Telemetry telemetry, int tagId)
    {
        return () ->
        {
            telemetry.addLine("looking for tag");
            telemetry.update();

            Limelight3A limelight = robot.limelight;

            LLResult result = limelight.getLatestResult();

            result.getPipelineIndex();
            boolean foundTag = result
                    .getFiducialResults()
                    .stream()
                    .anyMatch(fr -> fr.getFiducialId() == tagId);

            if(foundTag)
            {
                telemetry.addLine("found");
                telemetry.update();
                LLResultTypes.FiducialResult fiducialResult = result
                        .getFiducialResults()
                        .stream()
                        .filter(fr -> fr.getFiducialId() == tagId)
                        .findFirst()
                        .get();

                double tx = fiducialResult.getTargetXDegrees();
                if(tx > 10) {
                    telemetry.addLine("tag is to the right");
                    robot.getAutonomousRobot().mechAssembly.autonTurret.turnCW();
                    robot.getAutonomousRobot().mechAssembly.autonTurret.stop();
                    telemetry.update();

                }
                if(tx < -10) {
                    telemetry.addLine("tag is to the left");
                    robot.getAutonomousRobot().mechAssembly.autonTurret.turnCCW();
                    robot.getAutonomousRobot().mechAssembly.autonTurret.stop();
                    telemetry.update();
                }

            }
            return lookForTag(robot, telemetry,tagId);
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

            robot.getAutonomousRobot().mechAssembly.autonTurret.setPower(turnPower);

            return StraightDrive(robot, telemetry, -0.5, 0, 2000); // TEST
        };
    }

    public static IState ShootBalls(BillyRobot robot, Telemetry telemetry)
    {

        return() ->
        {
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

            return turnToAngle(robot, telemetry, 90);
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

                return IntakeBalls(robot,telemetry);
            }
        };
    }

    public static IState IntakeBalls(BillyRobot robot, Telemetry telemetry){
        return () -> {
            telemetry.clear();
            telemetry.addLine("intaking balls");

            robot.getAutonomousRobot().mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(3000);
            robot.getAutonomousRobot().driveTrain.drive(0,0.25,0);
            ThreadExtensions.TrySleep(2700);   //might have to change to back up intake
            robot.getAutonomousRobot().mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);

            //backing up
            robot.getAutonomousRobot().driveTrain.drive(0,-0.5,0);
            ThreadExtensions.TrySleep(1000);
            robot.getAutonomousRobot().driveTrain.drive(0.75,0,0);
            ThreadExtensions.TrySleep(1000);

            return ShootBalls(robot, telemetry);
        };
    }
}