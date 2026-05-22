package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;

public class Turret extends MechComponent
{

    public class AutonomousTurretBehaviors extends AutonomousComponentBehaviors
    {
        public AutonomousTurretBehaviors(Telemetry telemetry) {
            super(telemetry);
        }

        public void setPower(double power)
        {
            servo.setPower(power);
            reportData("Turret power", power);
        }
        public void turnCCW() { servo.setPower(1); reportStatus("Turret: CCW"); ThreadExtensions.TrySleep(100); }
        public void turnCW() { servo.setPower(-1); reportStatus("Turret: CW"); ThreadExtensions.TrySleep(100); }
        public void stop() { servo.setPower(0); reportStatus("Turret: stopped"); }
    }

    private AutonomousTurretBehaviors auton;

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

    /**
     * Stores the telemetry reference (via super) and lazily creates the autonomous
     * behaviors object. Previously {@code auton} was a field-initializer that ran
     * before telemetry was available, so the inner class had no telemetry reference
     * (Susan Zuo — two-phase initialization pattern).
     */
    @Override
    public void initializeTelemetry(Telemetry telemetry) {
        super.initializeTelemetry(telemetry);
        auton = new AutonomousTurretBehaviors(telemetry);
    }

    public void move(Gamepad gamepad)
    {
        strategy.move(servo, gamepad);
    }

    /**
     * Delegates to {@code telemetryStrategy} when present, otherwise writes
     * turret power directly. Previously empty (Susan Zuo — Bug #3: "No telemetry
     * data reported despite having telemetryStrategy"). Never flushes.
     */
    @Override
    void update()
    {
        if (telemetryStrategy != null) {
            telemetryStrategy.update(servo, telemetry);
        } else {
            telemetry.addData("turret power:", servo.getPower());
        }
    }

}
