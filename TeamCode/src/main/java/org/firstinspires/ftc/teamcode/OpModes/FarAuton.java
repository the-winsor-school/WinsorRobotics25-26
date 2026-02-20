//package org.firstinspires.ftc.teamcode.OpModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.AutonStrategies.FarStrategy;
//import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
//import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;
//
//@Autonomous(name = "far zone auton")
//public class FarAuton extends LinearOpMode {
//
//    private BillyRobot robot;
//    private IAutonStrategy autonStrategy;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot = new BillyRobot(hardwareMap, telemetry,);
//        autonStrategy = FarStrategy.far(robot.getAutonomousRobot(), telemetry, this);
//
//        telemetry.addLine("Far Auton Ready");
//        telemetry.update();
//
//        waitForStart();
//        autonStrategy.execute();
//    }
//}
