package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.ExampleAutonomousStrategies;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;


@Autonomous(name="Auton Example")
public class AutonomousExample extends LinearOpMode
{
    private Wildbots2025 robot;
    private IAutonStrategy autonStrategy;

    public AutonomousExample()
    {
        robot = new Wildbots2025(hardwareMap);
        autonStrategy = ExampleAutonomousStrategies.MecanumAutonDance(robot.getAutonomousRobot());
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        autonStrategy.execute();
    }
}
