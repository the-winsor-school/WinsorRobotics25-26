package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

public class rightSideAutonStrat
{
    public static IAutonStrategy right(BillyRobot.AutonomousMecanumRobot robot,
                                       Telemetry telemetry,
                                       LinearOpMode opMode)
    {
        return () ->
        {
            //moves out of zone
            //first ball
            robot.mechAssembly.autonFlywheel.shoot(0.79);
            ThreadExtensions.TrySleep(3500);
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonBallPusher.retractPusher();
            ThreadExtensions.TrySleep(1500);


            //second ball
            robot.mechAssembly.autonFlywheel.shoot(0.79);
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonIntake.reverseIntake();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonFlywheel.shoot(0.81);
            robot.mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonBallPusher.retractPusher();
            ThreadExtensions.TrySleep(1500);

            //third ball
            robot.mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonBallPusher.retractPusher();
            ThreadExtensions.TrySleep(200);

            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(1000);

            //turn
            robot.driveTrain.turnToAngle(70);
            //first ball
            robot.mechAssembly.autonIntake.startIntake();
            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(1000);
            robot.driveTrain.drive(0,-0.5,0);
            ThreadExtensions.TrySleep(500);

            //second ball
            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(500);
            robot.driveTrain.drive(0,-0.5,0);
            ThreadExtensions.TrySleep(500);

            //third ball
            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(500);
            robot.driveTrain.drive(0,-0.5,0);
            ThreadExtensions.TrySleep(1000);

            //turn again
            robot.driveTrain.turnToAngle(-70);

            //reverse intake a bit
            robot.mechAssembly.autonIntake.stopIntake();
            robot.mechAssembly.autonIntake.reverseIntake();
            ThreadExtensions.TrySleep(500);
            //shoot all three??

            robot.mechAssembly.autonFlywheel.shoot(0.79);
            ThreadExtensions.TrySleep(3500);
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonBallPusher.retractPusher();
            ThreadExtensions.TrySleep(1500);


            //second ball
            robot.mechAssembly.autonFlywheel.shoot(0.79);
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonIntake.reverseIntake();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonFlywheel.shoot(0.81);
            robot.mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonBallPusher.retractPusher();
            ThreadExtensions.TrySleep(1500);

            //third ball
            robot.mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonIntake.stopIntake();
            ThreadExtensions.TrySleep(200);
            ThreadExtensions.TrySleep(200);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(1000);
            robot.mechAssembly.autonBallPusher.retractPusher();
            ThreadExtensions.TrySleep(200);

            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(1000);
        };
    }
}
