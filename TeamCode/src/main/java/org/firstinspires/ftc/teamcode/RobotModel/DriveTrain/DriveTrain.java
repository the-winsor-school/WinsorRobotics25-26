package org.firstinspires.ftc.teamcode.RobotModel.DriveTrain;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class DriveTrain
{
    public abstract class AutonomousDriving {
        protected final Telemetry telemetry;
        public AutonomousDriving(Telemetry telemetry) {
            this.telemetry = telemetry;
        }
        public void reportStatus(String status) { telemetry.addLine(status); }
        public void reportData(String key, Object value) { telemetry.addData(key, value); }
    }
    public abstract <T extends AutonomousDriving> T getAutonomousDriving();

    public abstract void drive(Gamepad gamepad);

    protected Telemetry telemetry;

    public abstract void initializeTelemetry(Telemetry telemetry);
    public abstract void updateTelemetry();
}
