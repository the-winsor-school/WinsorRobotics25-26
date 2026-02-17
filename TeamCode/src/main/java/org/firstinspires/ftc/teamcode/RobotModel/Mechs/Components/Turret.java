package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends MechComponent
{

    public class AutonomousTurretBehaviors extends AutonomousComponentBehaviors
    {
        public void setPosition(double position)
        {
            servo.setPosition(position);
        }

    }

    private AutonomousTurretBehaviors auton = new AutonomousTurretBehaviors();

    @Override
    public AutonomousTurretBehaviors getAutonomousBehaviors()
    {
        return auton;
    }


    public interface TurretControlStrategy extends IControlStrategy
    {
        public void move(Servo servo, Gamepad gamepad);
    }

    public interface TurretTelemetryStrategy
    {
        public void update(Servo servo, Telemetry telemetry);
    }
    public Servo servo;

    protected TurretControlStrategy strategy;
    protected TurretTelemetryStrategy telemetryStrategy;

    public Turret(HardwareMap hardwareMap,
                String servoName,
                TurretControlStrategy strategy,
                TurretTelemetryStrategy telemetryStrategy)
    {
        super(strategy);
        servo = hardwareMap.get(Servo.class, servoName);
        this.strategy = strategy;
        this.telemetryStrategy = telemetryStrategy;
    }

    public void move(Gamepad gamepad)
    {
        strategy.move(servo, gamepad);
    }

    @Override
    public void update(Telemetry telemetry)
    {
        telemetryStrategy.update(servo, telemetry);
    }

}
