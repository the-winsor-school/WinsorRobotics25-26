package org.firstinspires.ftc.teamcode.RobotModel.DriveTrain;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * No-op DriveTrain implementation to avoid null checks when a robot has no drivetrain.
 */
public class NoDriveTrain extends DriveTrain
{
    private final NoAutonomousDriving autonomous = new NoAutonomousDriving();

    public class NoAutonomousDriving extends AutonomousDriving { }

    @SuppressWarnings("unchecked")
    @Override
    public <T extends AutonomousDriving> T getAutonomousDriving()
    {
        return (T) autonomous;
    }

    @Override
    public void drive(Gamepad gamepad)
    {
        // intentionally empty
    }

    @Override
    public void updateTelemetry(Telemetry telemetry)
    {
        // intentionally empty
    }
}
