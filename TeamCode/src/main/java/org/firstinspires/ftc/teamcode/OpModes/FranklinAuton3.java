package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AutonStrategies.FranklinAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.FranklinBlueAuton;
import org.firstinspires.ftc.teamcode.AutonStrategies.FranklinStateAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;


/**
 * TODO:  Is this RED or is this BLUE?
 */
@Autonomous(name = "Franklin Red Auton From Close Launch Zone")
public class FranklinAuton3 extends LinearOpMode {
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
                FranklinStateAutonStrategy.ShootAtRedFromGoal(
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
