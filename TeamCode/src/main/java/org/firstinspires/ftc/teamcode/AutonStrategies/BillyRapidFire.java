package org.firstinspires.ftc.teamcode.AutonStrategies;


import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.IState;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.BillyMA;


public class BillyRapidFire extends StateMachine implements BillyMA.BillyAssemblyStrategy
{
    private final BillyMA.AutonomousBillyMA mechAssembly;
    private int ballCount;
    private final Telemetry telemetry;

    /**
     * 
     * @param ma The mechAssembly of BillyRapidFire
     * @param ballCount The number of balls that BillyRapidFire will launch
     * @param tm Telemetry is good and necessary! yay
     */
    public BillyRapidFire(BillyMA.AutonomousBillyMA ma, int ballCount, Telemetry tm)
    {
        mechAssembly = ma; //dependency
        telemetry = tm;
        reset(ballCount);
    }

    /**
     * Resets BillyRapidFire
     * @param ballCount The number of balls that BillyRapidFire will launch
     */
    public void reset(int ballCount)
    {
        this.ballCount = ballCount;
        currentState = startShooter();
    }

    /**
     * Starts the flywheel shooter
     * @return Sets the flywheel to 0.6 power and waits for 2200 milliseconds, then goes to fire()
     */
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

    /**
     * Pushes the balls into the flywheel shooter
     * @return Pushes the balls and waits for 1000 milliseconds, then goes to retract()
     */
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

    /**
     * Retracts the ball pusher
     * @return Retracts the ball pusher servo and waits for 1000 milliseconds.
     * If ballCount > 0, goes to fire(). Otherwise, goes to stopShooter()
     */
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

    /**
     * Stops the flywheel shooter
     * @return Stops the flywheel shooter, then returns null.
     */
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

    //actually I don't think this one does anything
    @Override
    public void execute(BillyMA mechAssembly, Gamepad gamepad) {

    }
}
