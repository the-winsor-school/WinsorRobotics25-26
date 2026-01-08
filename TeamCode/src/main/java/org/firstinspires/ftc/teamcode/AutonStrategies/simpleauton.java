package org.firstinspires.ftc.teamcode.AutonStrategies;

import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;

public class simpleauton
{
    /** make it dance! */
    public static IAutonStrategy simple(BillyRobot.AutonomousMecanumRobot robot)
    {
        return () ->
        {
            robot.driveTrain.drive(0,-1,0);
            ThreadExtensions.TrySleep(100);

            robot.mechAssembly.autonFlywheel.StartShoot();
            ThreadExtensions.TrySleep(100);

            robot.mechAssembly.autonBallPusher.pushBalls();
        };
    }
}
