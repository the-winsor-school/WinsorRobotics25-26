package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PusherServo extends MechComponent {

    public class AutonomousBallPusherBehaviors extends AutonomousComponentBehaviors {
        public void pushBalls() {
            servoR.setPosition(0.22);
            servoL.setPosition(0.22);
        }

        public void setPosition(double position) {
            servoR.setPosition(position);
            servoL.setPosition(position);
        }

        public void retractPusher() {
            servoR.setPosition(0.0);
            servoL.setPosition(0.0);
        }
    }

    public interface BallPusherControlStrategy extends IControlStrategy {
        void controlPusher(Servo servoR, Servo servoL, Gamepad gamepad);
    }

    private final Servo servoR;
    private final Servo servoL;
    private final BallPusherControlStrategy strategy;
    private final AutonomousBallPusherBehaviors auton = new AutonomousBallPusherBehaviors();

    public PusherServo(HardwareMap hardwareMap, String servoNameR, String servoNameL, BallPusherControlStrategy strategy) {
        super(strategy);
        this.servoR = hardwareMap.get(Servo.class, servoNameR);
        this.servoL = hardwareMap.get(Servo.class, servoNameL);
        this.strategy = strategy;
    }

    @Override
    public AutonomousBallPusherBehaviors getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.controlPusher(servoR, servoL, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addData("Right Pusher Position", "%.2f", servoR.getPosition());
        telemetry.addData("Left Pusher Position", "%.2f", servoL.getPosition());
    }
}