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
            //start at a position on the white tape at the back of the field where it can shoot three preloaded balls
            robot.mechAssembly.autonFlywheel.StartShoot();
            ThreadExtensions.TrySleep(3000);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonFlywheel.StopShoot();

            //drive to first row
            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(500);
            robot.driveTrain.turnToAngle(70);

            //spinny intake collects 3 balls there
            robot.mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(3000);
            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonIntake.stopIntake();

            //return to reg position
            robot.driveTrain.drive(-0.5,-0.5,0);
            ThreadExtensions.TrySleep(500);
            robot.driveTrain.turnToAngle(-70);

            //shoot balls
            robot.mechAssembly.autonFlywheel.StartShoot();
            ThreadExtensions.TrySleep(3000);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonFlywheel.StopShoot();

            //go to second row of balls
            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(1000);
            robot.driveTrain.turnToAngle(70);

            //spinny intake collects 3 balls there
            robot.mechAssembly.autonIntake.startIntake();
            ThreadExtensions.TrySleep(3000);
            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonIntake.stopIntake();

            //go back to og pos
            robot.driveTrain.drive(-0.5,-0.5,0);
            ThreadExtensions.TrySleep(1000);
            robot.driveTrain.turnToAngle(-70);

            //shoot
            robot.mechAssembly.autonFlywheel.StartShoot();
            ThreadExtensions.TrySleep(3000);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonBallPusher.pushBalls();
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonFlywheel.StopShoot();

            //go to ending position
            robot.driveTrain.drive(0,0.5,0);
            ThreadExtensions.TrySleep(1500);
        };
    }
}
