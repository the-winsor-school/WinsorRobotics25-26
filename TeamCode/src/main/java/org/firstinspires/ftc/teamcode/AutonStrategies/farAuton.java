package org.firstinspires.ftc.teamcode.AutonStrategies;

import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

public class farAuton
{
    //shoots preloaded and moves out of zone
    public static IAutonStrategy fwwar(BillyRobot.AutonomousMecanumRobot robot)
    {
        return () ->
        {
            //shoots
            robot.mechAssembly.autonFlywheel.StartShoot();
            /**CHANGE TMR**/
            ThreadExtensions.TrySleep(3000);
            robot.mechAssembly.autonBallPusher.pushBalls();
            /**CHANGE TMR**/
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonFlywheel.StopShoot();

            //moves out of zone
            robot.driveTrain.drive(0,0.5,0);
            /**CHANGE TMR**/
            ThreadExtensions.TrySleep(200);
        };
    }
}
