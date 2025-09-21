package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw extends MechComponent
{

    public class AutonomousClawBehaviors extends AutonomousComponentBehaviors
    {
        public void open()
        {
            servo.setPower(1);
        }

        public void close()
        {
            servo.setPower(-1);
        }

    }

    private AutonomousClawBehaviors auton = new AutonomousClawBehaviors();

    @Override
    public AutonomousClawBehaviors getAutonomousBehaviors()
    {
        return auton;
    }


    public interface ClawControlStrategy extends IControlStrategy
    {
        public void chomp(CRServo servo, Gamepad gamepad);
    }

    public interface ClawTelemetryStrategy
    {
        public void update(CRServo servo, Telemetry telemetry);
    }
    public CRServo servo;

    protected ClawControlStrategy strategy;
    protected ClawTelemetryStrategy telemetryStrategy;

    public Claw(HardwareMap hardwareMap,
                String servoName,
                ClawControlStrategy strategy,
                ClawTelemetryStrategy telemetryStrategy)
    {
        super(strategy);
        servo = hardwareMap.get(CRServo.class, servoName);
        this.strategy = strategy;
        this.telemetryStrategy = telemetryStrategy;
    }

    public void move(Gamepad gamepad)
    {
        strategy.chomp(servo, gamepad);
    }

    @Override
    public void update(Telemetry telemetry)
    {
        telemetryStrategy.update(servo, telemetry);
    }

}
