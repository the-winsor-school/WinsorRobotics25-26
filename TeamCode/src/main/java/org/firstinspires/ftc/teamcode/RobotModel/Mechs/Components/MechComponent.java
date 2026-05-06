package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class MechComponent
{
    public abstract class AutonomousComponentBehaviors {
        protected final Telemetry telemetry;
        public AutonomousComponentBehaviors(Telemetry telemetry) {
            this.telemetry = telemetry;
        }
        public void reportStatus(String status) { telemetry.addLine(status); }
        public void reportData(String key, Object value) { telemetry.addData(key, value); }
    }
    public abstract <T extends AutonomousComponentBehaviors> T getAutonomousBehaviors();
    protected interface IControlStrategy { }

    protected IControlStrategy strategy;
    protected Telemetry telemetry;

    protected MechComponent(IControlStrategy strategy)
    {
        this.strategy = strategy;
    }

    public void initializeTelemetry(Telemetry telemetry)
    {
        this.telemetry = telemetry;
    }

    abstract void move(Gamepad gamepad);
    abstract void update();
}
