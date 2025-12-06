package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.FranklinBlueAuton;
import org.firstinspires.ftc.teamcode.AutonStrategies.FranklinStateAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;


@Autonomous(name = "Franklin Target Tag 24")
public class FranklinAuton2 extends LinearOpMode {
    private FranklinRobot robot;
    private IAutonStrategy autonStrategy;
    private final int TARGET_TAG_ID = 24; // Red

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new FranklinRobot(hardwareMap);
        autonStrategy = 
            /*FranklinAutonStrategy.TargetSpecificAprilTag(
                robot.getAutonomousRobot(),
                robot,
                TARGET_TAG_ID,
                telemetry
        );*/
            FranklinBlueAuton.ShootAtBlue(
                robot,
                telemetry,
            this);

        telemetry.addLine("Franklin Autonomous Ready");
        telemetry.addData("Target AprilTag ID", TARGET_TAG_ID);
        telemetry.update();

        waitForStart();
        autonStrategy.execute();
    }

}

