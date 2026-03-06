package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

@Autonomous(name = "Second Example Auton")
    public class SecondExampleAuton
        extends LinearOpMode
        implements IAutonStrategy {

    private BillyRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BillyRobot(hardwareMap, telemetry, 20);
        telemetry.addLine("auton ready");
        telemetry.update();
        waitForStart();
        execute();
    }

    @Override
    public void execute() {

    }

    private IState findTags() {
        
    }
}
