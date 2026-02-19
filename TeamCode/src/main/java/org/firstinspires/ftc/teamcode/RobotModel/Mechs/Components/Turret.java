package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends MechComponent
{

    public class AutonomousTurretBehaviors extends AutonomousComponentBehaviors
    {
        public void setPower(double power)
        {
            servo.setPower(power);
        }
        public void turnCCW() { servo.setPower(1); }
        public void turnCW() { servo.setPower(-1); }
    }

    private AutonomousTurretBehaviors auton = new AutonomousTurretBehaviors();

    @Override
    public AutonomousTurretBehaviors getAutonomousBehaviors()
    {
        return auton;
    }


    public interface TurretControlStrategy extends IControlStrategy
    {
        public void move(CRServo servo, Gamepad gamepad);
    }

    public interface TurretTelemetryStrategy
    {
        public void update(CRServo servo, Telemetry telemetry);
    }
    public CRServo servo;

    protected TurretControlStrategy strategy;
    protected TurretTelemetryStrategy telemetryStrategy;

    public Turret(HardwareMap hardwareMap,
                String servoName,
                TurretControlStrategy strategy,
                TurretTelemetryStrategy telemetryStrategy)
    {
        super(strategy);
        servo = hardwareMap.get(CRServo.class, servoName);
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
