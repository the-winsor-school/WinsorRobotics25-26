package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpinnyIntake extends MechComponent
{
    public class AutonomousIntakeBehaviors extends MechComponent.AutonomousComponentBehaviors
    {
        public void startIntake()
        {
            intake.setPower(1);
        }

        public void stopIntake()
        {
            intake.setPower(0);
        }

        public void reverseIntake()
        {
            intake.setPower(-1);
        }
    }


    public interface SpinnyIntakeControlStrategy extends IControlStrategy
    {
        /**
         * abstract definition of how a Spinny Intake gets controlled.
         * this Implementation should translate the Gamepad Input into motor movement
         * of the Spinny Intake.
         * The name of this method does not matter at all~ so make it descriptive in the
         * context of the Spinny Intake!
         * @param motor the `intake` motor will be passed here.
         * @param gamepad the gamepad will be passed down from the MechAssembly to this strategy
         */
        void nomNomNom(DcMotor motor, Gamepad gamepad);
    }
    private final DcMotor intake;

    protected SpinnyIntakeControlStrategy strategy;

    private final AutonomousIntakeBehaviors auton = new AutonomousIntakeBehaviors();;

    public SpinnyIntake(
            HardwareMap hardwareMap,
            String motorName,
            SpinnyIntakeControlStrategy strategy)
    {
        super(strategy);
        intake = hardwareMap.get(DcMotor.class, motorName);
        // pointer magic ... just be clear that THIS strategy is a SpinnyIntakeStrategy.
        this.strategy = (SpinnyIntakeControlStrategy) super.strategy;
    }

    @Override
    public AutonomousIntakeBehaviors getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad)
    {
        strategy.nomNomNom(intake, gamepad);
    }

    @Override
    void update(Telemetry telemetry) {

    }
}
