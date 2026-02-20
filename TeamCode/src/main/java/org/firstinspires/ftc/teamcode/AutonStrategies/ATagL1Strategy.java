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

    public static IState lookForTag(BillyRobot robot, Telemetry telemetry, int tagId) {
        return () ->
        {
            telemetry.clear();
            telemetry.addLine("looking for tag");
            telemetry.update();

            Limelight3A limelight = robot.limelight;

            LLResult result = limelight.getLatestResult();

            result.getPipelineIndex();
            boolean foundTag = result
                    .getFiducialResults()
                    .stream()
                    .anyMatch(fr -> fr.getFiducialId() == tagId);

            if (foundTag) {
                LLResultTypes.FiducialResult fiducialResult = result
                        .getFiducialResults()
                        .stream()
                        .filter(fr -> fr.getFiducialId() == tagId)
                        .findFirst()
                        .get();

                double tx = fiducialResult.getTargetXDegrees();
                if (tx > 0) {
                    robot.getAutonomousRobot().mechAssembly.autonTurret.turnCCW();
                    telemetry.addLine("tag is to the right");
                    telemetry.update();
                }
                if (tx < 0) {
                    robot.getAutonomousRobot().mechAssembly.autonTurret.turnCW();
                    telemetry.addLine("tag is to the left");
                    telemetry.update();
                }
            }
            return lookForTag(robot, telemetry, tagId);
        };
    }

    public static IState shootBalls(BillyRobot robot, Telemetry telemetry)
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

            return null;
        };
    }
}
