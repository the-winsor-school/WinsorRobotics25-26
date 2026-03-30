package org.firstinspires.ftc.teamcode.AutonStrategies;

import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.BillyMA;

public class BillyRapidFire extends StateMachine
{
    private final BillyMA.AutonomousBillyMA mechAssembly;
    private int ballCount;
    public BillyRapidFire(BillyMA.AutonomousBillyMA ma, int ballCount)
    {
        mechAssembly = ma;
        reset(ballCount);
    }
    public void reset(int ballCount)
    {
        this.ballCount = ballCount;
        currentState = startShooter();
    }

    public IState startShooter() {
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
        return() ->
        {
            mechAssembly
                    .autonFlywheel
                    .StopShoot();
            return null;
        };
    }
}
