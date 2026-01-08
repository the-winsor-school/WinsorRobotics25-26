package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelMotor extends MechComponent {

    public class AutonomousFlywheelBehaviors extends AutonomousComponentBehaviors {
        public void startFlywheel() {
            motor.setPower(0.8);
            isRunning = true;
        }

        public void stopFlywheel() {
            motor.setPower(0);
            isRunning = false;
        }
    }

    public interface FlywheelControlStrategy extends IControlStrategy {
        void controlFlywheel(FlywheelMotor flywheelMotor, Gamepad gamepad, boolean isRunning);
    }

    private final DcMotor motor;
    private final FlywheelControlStrategy strategy;
    private final AutonomousFlywheelBehaviors auton = new AutonomousFlywheelBehaviors();

    private boolean isRunning = false;
    private boolean wasYPressed = false;

    public FlywheelMotor(HardwareMap hardwareMap, String motorName, FlywheelControlStrategy strategy) {
        super(strategy);
        this.motor = hardwareMap.get(DcMotor.class, motorName);
        this.strategy = strategy;
    }

    @Override
    public AutonomousFlywheelBehaviors getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.controlFlywheel(this, gamepad, isRunning);
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addData("Flywheel Status", isRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Flywheel Power", "%.2f", motor.getPower());
    }

    public void toggleFlywheel() {
        if (isRunning) {
            motor.setPower(0);
            isRunning = false;
        } else {
            motor.setPower(0.8);
            isRunning = true;
        }
    }

    public boolean wasYPressed() {
        return wasYPressed;
    }

    public void setYPressed(boolean pressed) {
        wasYPressed = pressed;
    }
}