package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutonStrategies.FranklinAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;


@Autonomous(name = "Franklin AprilTag Auton")
public class FranklinAuton extends LinearOpMode {
    private FranklinRobot robot;
    private IAutonStrategy autonStrategy;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new FranklinRobot(hardwareMap);
        autonStrategy = FranklinAutonStrategy.AprilTagNavigationStrategy(
                robot.getAutonomousRobot(),
                robot
        );

        telemetry.addLine("Franklin AprilTag Autonomous Ready");
        telemetry.update();

        waitForStart();
        autonStrategy.execute();
    }
}
