package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonStrategies.IAutonStrategy;
import org.firstinspires.ftc.teamcode.AutonStrategies.LimelightAutoTarget;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.BillyRobot;

@Autonomous(name = "Billy Example")
public class BillyExampleAuton
        extends LinearOpMode
        implements IAutonStrategy

{
    private BillyRobot robot;

    public BillyExampleAuton()
    {
        robot = new BillyRobot(hardwareMap, telemetry, 20);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("auton ready");
        telemetry.update();
        waitForStart();
        execute();

    }
    private final LimelightAutoTarget targeter =
            new LimelightAutoTarget(
                    robot.limelight, //why is this yellow underlined? most likely not an issue
                    robot.getAutonomousRobot().mechAssembly.autonTurret,
                    telemetry,
                    20);
    @Override
    public void execute() {
        IState currentState = step1;
        while(opModeIsActive() && currentState !=null)
        {
            targeter.updateState();
            currentState = currentState.execute();
            idle();
        }

    }
    
    private ElapsedTime elapsedTime;
    
    private IState doAndWait(Runnable action, int millis, IState nextState) {
        return () -> //lambda expression
        {
            if (elapsedTime == null) 
            {
                elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                action.run();
            }

            if (elapsedTime.milliseconds() < millis)
                return doAndWait(action, millis, nextState);
            elapsedTime = null;
            return nextState;
        };
    }

    //private IState step4;

    //private IState step3;

    //private IState step2;

    //private IState step1;
    private IState step12 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .driveTrain
                            .drive(0,0.5, 0),
                    1500,
                    null
            );
    private IState step11 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonBallPusher
                            .retractPusher(),
                    100,
                    step12
            );
    private IState step10 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonBallPusher
                            .pushBalls(),
                    100,
                    step11
            );
    private IState step9 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonIntake
                            .stopIntake(),
                    100,
                    step10
            );
    private IState step8 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonIntake
                            .startIntake(),
                    1000,
                    step9
            );
    private IState step7 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonBallPusher
                            .retractPusher(),
                    1500,
                    step8
            );
    private IState step6 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonBallPusher
                            .pushBalls(),
                    1000,
                    step7
            );
    private IState step5 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonIntake
                            .stopIntake(),
                    100,
                    step6
            );
    private IState step4 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonIntake
                            .startIntake(),
                    100,
                    step5
            );
    private IState step3 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonBallPusher
                            .retractPusher(),
                    1500,
                    step4
            );
    private IState step2 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonBallPusher
                            .pushBalls(),
                    1000,
                    step3
            );
    private IState step1 =
            doAndWait(
                    () -> robot.getAutonomousRobot()
                            .mechAssembly
                            .autonFlywheel
                            .shoot(0.6),
                    2200,
                    step2
            );

}
