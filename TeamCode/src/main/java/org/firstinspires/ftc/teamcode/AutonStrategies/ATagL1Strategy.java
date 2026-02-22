package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

public class ATagL1Strategy {
    public static IAutonStrategy track(BillyRobot robot, Telemetry telemetry, LinearOpMode opMode, int tagID) {
        return () ->
        {

            IState turretState = lookForTag(robot, telemetry,tagID);
            IState driveState = shootBalls(robot, telemetry);

            while (opMode.opModeIsActive() && driveState != null) {
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

    public static IState shootBalls(BillyRobot robot, Telemetry telemetry)
    {

        return() ->
        {
            robot.getAutonomousRobot().mechAssembly.autonFlywheel.shoot(0.46);
            ThreadExtensions.TrySleep(2000);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.setPosition(0);
            ThreadExtensions.TrySleep(1500);
            robot.getAutonomousRobot().mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(100);
            robot.getAutonomousRobot().mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(100);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.getAutonomousRobot().mechAssembly.autonBallPusher.setPosition(0);
            ThreadExtensions.TrySleep(1500);

            robot.getAutonomousRobot().mechAssembly.autonIntake.stopIntake();

            robot.getAutonomousRobot().driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(1500);

            return null;
        };
    }
}
