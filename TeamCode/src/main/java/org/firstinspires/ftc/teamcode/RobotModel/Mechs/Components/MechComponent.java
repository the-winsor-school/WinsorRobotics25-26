package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class MechComponent
{
    /**
     * Override with an implemenation that applies to the Mech in question.
     */
    public abstract class AutonomousComponentBehaviors { }
    public abstract <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors();
    protected interface IControlStrategy { }

    /**
     * a mech component must have a control strategy!
     */
    protected IControlStrategy strategy;

    protected MechComponent(IControlStrategy strategy)
    {
        this.strategy = strategy;
    }

    /**
     * the implementation of this method should use the
     * Control Strategy defined in the implementing class
     * @param gamepad gamepad passed down from the MechAssembly
     */
    abstract void move(Gamepad gamepad);
    abstract void update(Telemetry telemetry);
}
