package org.firstinspires.ftc.teamcode.AutonStrategies;

import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

public class closeAuton {
    public static IAutonStrategy close(BillyRobot.AutonomousMecanumRobot robot)
    {
        return() ->
        {
            //backs up
            robot.driveTrain.drive(0,-0.5,0);
            /**CHANGE TMR**/
            ThreadExtensions.TrySleep(200);

            //shoots
            robot.mechAssembly.autonFlywheel.StartShoot();
            /**CHANGE TMR**/
            ThreadExtensions.TrySleep(3000);
            robot.mechAssembly.autonBallPusher.pushBalls();
            /**CHANGE TMR**/
            ThreadExtensions.TrySleep(100);
            robot.mechAssembly.autonFlywheel.StopShoot();

            //moves out of zone
            robot.driveTrain.drive(0,-0.5,0);
            /**CHANGE TMR**/
            ThreadExtensions.TrySleep(200);
        };
    }
}
