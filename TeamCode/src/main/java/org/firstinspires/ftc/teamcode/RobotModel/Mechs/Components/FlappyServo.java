package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

public class FlappyServo extends MechComponent{

    public  interface FlappyServoControlStrategy extends IControlStrategy {
        public void UCS (Servo servo, Gamepad gamepad);
    }

    public FlappyServo(
            HardwareMap hardwareMap,
            String servoName,
            FlappyServoControlStrategy strategy) {
        super(strategy);
        flappydoodle = hardwareMap.get(Servo.class, servoName);
        FlappyCS = strategy;
    }

    public class AutonomousFlappyBehavior extends AutonomousComponentBehaviors {
        public void FlappyUp(){flappydoodle.setPosition(0); }

        public void FlappyDown(){
            flappydoodle.setPosition(60);
        }

    }

    @Override
    public AutonomousFlappyBehavior getAutonomousBehaviors() {
        return auton;
    }

    private final Servo flappydoodle;
    private final AutonomousFlappyBehavior auton = new AutonomousFlappyBehavior();
    private final FlappyServoControlStrategy FlappyCS;
    @Override
    public void move(Gamepad gamepad) {
        FlappyCS.UCS(flappydoodle, gamepad);
    }

    public void update(Telemetry telemetry) {
        telemetry.addData("Servo position ", flappydoodle.getPosition());
    }
}
