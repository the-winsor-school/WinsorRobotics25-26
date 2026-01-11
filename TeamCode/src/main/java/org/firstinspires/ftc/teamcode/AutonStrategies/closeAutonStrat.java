package org.firstinspires.ftc.teamcode.AutonStrategies;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

public class closeAutonStrat {
    public static IAutonStrategy close(BillyRobot.AutonomousMecanumRobot robot,
                                       Telemetry telemetry,
                                       LinearOpMode opMode)
    {
        return() ->
        {
            //backs up
            robot.driveTrain.drive(0,-0.5,0);
            /**CHANGE TMR**/
            ThreadExtensions.TrySleep(2000);

            robot.driveTrain.drive(-0.5,0,0);
            /**CHANGE TMR**/
            ThreadExtensions.TrySleep(2000);
        };
    }
}
