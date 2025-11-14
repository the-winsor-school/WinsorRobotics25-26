package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutonStrategies.FranklinAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;


@Autonomous(name = "Franklin Target Tag 24")
public class FranklinAuton extends LinearOpMode {
    private FranklinRobot robot;
    private IAutonStrategy autonStrategy;
    private final int TARGET_TAG_ID = 24; // Red

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new FranklinRobot(hardwareMap);
        autonStrategy = FranklinAutonStrategy.TargetSpecificAprilTag(
                robot.getAutonomousRobot(),
                robot,
                TARGET_TAG_ID
        );

        telemetry.addLine("Franklin Autonomous Ready");
        telemetry.addData("Target AprilTag ID", TARGET_TAG_ID);
        telemetry.addData("Range", range);
        telemetry.update();

        waitForStart();
        autonStrategy.execute();
    }
}
