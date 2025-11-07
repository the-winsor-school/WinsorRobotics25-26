package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlappyServo extends MechComponent{

    public  interface FlappyServoControlStrategy extends IControlStrategy {
        public void UCS (FlappyServo servo, Gamepad gamepad);
    }

    public FlappyServo(
            HardwareMap hardwareMap,
            String servoName,
            String upperSensorName,
            String lowerSensorName,
            FlappyServoControlStrategy strategy) {
        super(strategy);
        flappydoodle = hardwareMap.get(CRServo.class, servoName);
        upperSensor = hardwareMap.get(TouchSensor.class, upperSensorName);
        lowerSensor = hardwareMap.get(TouchSensor.class, lowerSensorName);
        FlappyCS = strategy;
    }

    public class AutonomousFlappyBehavior extends AutonomousComponentBehaviors {
        public void FlappyPos(){flappydoodle.setPower(0.25); }
        public void FlappyStop(){flappydoodle.setPower(0);
        }
        public void FlappyNeg(){flappydoodle.setPower(-0.25);
        }

    }

    public void setPower(double power){

        if ((power > 0 && !canGoForward() )|| (power < 0 && !canGoReverse())) {
            power = 0;
        }

        flappydoodle.setPower(power);
    }

    @Override
    public AutonomousFlappyBehavior getAutonomousBehaviors() {
        return auton;
    }

    private final CRServo flappydoodle;
    private final TouchSensor upperSensor;
    private final TouchSensor lowerSensor;
    private final AutonomousFlappyBehavior auton = new AutonomousFlappyBehavior();
    private final FlappyServoControlStrategy FlappyCS;
    @Override
    public void move(Gamepad gamepad) {
        FlappyCS.UCS(this, gamepad);
    }

    public void update(Telemetry telemetry) {
        // telemetry.addData("Servo position ", flappydoodle.getPosition());
    }


    /**
     *
     * @return true if the forward sensor is not pressed
     */
    public boolean canGoForward()
    {
        return ! upperSensor.isPressed();
    }

    /**
     *
     * @return true if the reverse sensor is not pressed
     */
    public boolean canGoReverse()
    {
        return ! lowerSensor.isPressed();
    }
}
