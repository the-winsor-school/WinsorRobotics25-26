package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

public class Combined {
    public static IAutonStrategy track(BillyRobot robot, Telemetry telemetry, LinearOpMode opMode)
    {
        return () ->
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

            IState currentState = lookForTag(robot, telemetry);

            while (opMode.opModeIsActive() && currentState != null )
            {
                currentState = currentState.execute();
                opMode.idle();
            }
        };
    }

    public static IState lookForTag(BillyRobot robot, Telemetry telemetry)
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
    }

    public static IState turnToTag(BillyRobot robot, Telemetry telemetry, Double angle)
    {
        if (angle == 0)
        {
            return lookForTag(robot, telemetry);
        }

        robot.getAutonomousRobot().driveTrain.turnToAngle(angle);

        ThreadExtensions.TrySleep(100);

        return lookForTag(robot, telemetry);
    }
}
