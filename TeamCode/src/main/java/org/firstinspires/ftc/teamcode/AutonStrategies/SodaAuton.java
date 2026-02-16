/* OpModes/SodaAuton.java */
package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.SodaAutoAimStrategy;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.SodapopRobot;

@Autonomous(name = "Soda Auto-Aim", group = "Competition")
public class SodaAuton extends LinearOpMode {

    private SodapopRobot robot;
    private IAutonStrategy autonStrategy;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SodapopRobot(hardwareMap);
        autonStrategy = SodaAutoAimStrategy.AutoAimAndShoot(robot, telemetry, this);

        telemetry.addLine("Soda Autonomous Ready");
        telemetry.addLine("Strategy: Auto-Aim and Shoot");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            autonStrategy.execute();
        }
    }
}
