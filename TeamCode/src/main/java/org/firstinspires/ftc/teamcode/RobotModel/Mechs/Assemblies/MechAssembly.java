package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.MechComponent;

/**
 * Mech Assembly is a Composite of all the mech components
 * in a robot.  Inheritors should have discrete Mech Component
 * implementations that it hands down instructions to.
 */
public abstract class MechAssembly
{
    protected interface IAssemblyStrategy { }
    protected MechAssembly.IAssemblyStrategy strategy;

    public abstract class AutonomousMechBehaviors { }
    public abstract <T extends AutonomousMechBehaviors> T getAutonomousBehaviors();

    /**
     * Pass along gamepad instructions to subcomponents.
     * @param gamepad the Gamepad
     */
    public abstract void giveInstructions(Gamepad gamepad);
    public abstract void updateTelemetry(Telemetry telemetry);
}
