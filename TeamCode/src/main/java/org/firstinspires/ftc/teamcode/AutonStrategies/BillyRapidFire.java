package org.firstinspires.ftc.teamcode.AutonStrategies;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.BillyMA;


public class BillyRapidFire extends StateMachine
{
    private final BillyMA.AutonomousBillyMA mechAssembly;
    private int ballCount;
    private final Telemetry telemetry;
    public BillyRapidFire(BillyMA.AutonomousBillyMA ma, int ballCount, Telemetry tm)
    {
        mechAssembly = ma; //dependency
        telemetry = tm;
        reset(ballCount);
    }
    public void reset(int ballCount)
    {
        this.ballCount = ballCount;
        currentState = startShooter();
    }


    public IState startShooter() {
        telemetry.addLine("startShooter");
        telemetry.update();
        return
                doAndWait(
                        () -> mechAssembly
                                .autonFlywheel
                                .setPower(0.6),
                        2200,
                        fire()
                );


    }
    public IState fire()
    {
        telemetry.addLine("fire");
        telemetry.update();
        return
                doAndWait(
                        mechAssembly
                                .autonBallPusher
                                ::pushBalls,
                        1000,
                        retract()
                );
    }
    public IState retract()
    {
        telemetry.addLine("retract");
        telemetry.update();
        return
                doAndWait(
                        mechAssembly
                                .autonBallPusher
                                ::retractPusher,
                        1000,
                        --ballCount > 0
                                ? fire ()
                                : stopShooter()


                );
    }
    public IState stopShooter()
    {
        telemetry.addLine("stopShooter");
        telemetry.update();
        return() ->
        {
            mechAssembly
                    .autonFlywheel
                    .StopShoot();
            return null;
        };
    }

}
