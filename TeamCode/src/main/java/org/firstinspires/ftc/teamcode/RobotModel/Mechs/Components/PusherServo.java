package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PusherServo extends MechComponent {

    public class AutonomousBallPusherBehaviors extends AutonomousComponentBehaviors {
        public void pushBalls() {
            servo.setPosition(0.22);  // Push position
        }

        public void retractPusher() {
            servo.setPosition(0.0);   // Rest position
        }
    }

    public interface BallPusherControlStrategy extends IControlStrategy {
        void controlPusher(Servo servo, Gamepad gamepad);
    }

    private final Servo servo;
    private final BallPusherControlStrategy strategy;
    private final AutonomousBallPusherBehaviors auton = new AutonomousBallPusherBehaviors();

    public PusherServo(HardwareMap hardwareMap, String servoName, BallPusherControlStrategy strategy) {
        super(strategy);
        this.servo = hardwareMap.get(Servo.class, servoName);
        this.strategy = strategy;
    }

    @Override
    public AutonomousBallPusherBehaviors getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.controlPusher(servo, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addData("Ball Pusher Position", "%.2f", servo.getPosition());
    }
}