package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * No-op MechAssembly implementation to serve as a safe default when no mechanisms are present.
 */
public class NoMechAssembly extends MechAssembly
{
    private final NoAutonomousMechBehaviors autonomous = new NoAutonomousMechBehaviors();

    public class NoAutonomousMechBehaviors extends AutonomousMechBehaviors { }

    @SuppressWarnings("unchecked")
    @Override
    public <T extends AutonomousMechBehaviors> T getAutonomousBehaviors()
    {
        return (T) autonomous;
    }

    @Override
    public void giveInstructions(Gamepad gamepad)
    {
        // intentionally empty
    }

    @Override
    public void updateTelemetry(Telemetry telemetry)
    {
        // intentionally empty
    }
}
