package org.firstinspires.ftc.teamcode.RobotModel.DriveTrain;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class DriveTrain
{
    public abstract class AutonomousDriving { }
    public abstract <T extends AutonomousDriving> T getAutonomousDriving();

    public abstract void drive(Gamepad gamepad);

    public abstract void updateTelemetry(Telemetry telemetry);
}
