/* RobotModel/Mechs/Components/SodaColorSensorArray.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaColorSensorArray extends MechComponent {

    public interface SodaColorSensorStrategy extends IControlStrategy {
        void processColorData(ColorSensor[] sensors, Gamepad gamepad);
    }

    public SodaColorSensorArray(HardwareMap hardwareMap,
                                String[] sensorNames,
                                SodaColorSensorStrategy strategy) {
        super(strategy);
        this.colorSensors = new ColorSensor[sensorNames.length];
        for (int i = 0; i < sensorNames.length; i++) {
            this.colorSensors[i] = hardwareMap.get(ColorSensor.class, sensorNames[i]);
        }
        this.strategy = strategy;
    }

    public class AutonomousColorSensors extends AutonomousComponentBehaviors {

        /**
         * Check if a ball is detected at a specific position
         * Threshold: if intensity (alpha) is high, ball is present
         * @param sensorIndex 0, 1, or 2
         * @return true if artifact detected
         */
        public boolean isBallDetected(int sensorIndex) {
            if (sensorIndex < 0 || sensorIndex >= colorSensors.length) return false;
            ColorSensor sensor = colorSensors[sensorIndex];
            // Alpha > 500 means something is blocking the sensor
            return sensor.alpha() > 500;
        }

        /**
         * Get RGB values from a sensor
         */
        public int[] getRGB(int sensorIndex) {
            if (sensorIndex < 0 || sensorIndex >= colorSensors.length) {
                return new int[]{0, 0, 0};
            }
            ColorSensor sensor = colorSensors[sensorIndex];
            return new int[]{sensor.red(), sensor.green(), sensor.blue()};
        }

        /**
         * Check if any position has a ball
         */
        public boolean anyBallDetected() {
            for (int i = 0; i < colorSensors.length; i++) {
                if (isBallDetected(i)) return true;
            }
            return false;
        }

        /**
         * Get the index of the first detected ball (0, 1, or 2)
         */
        public int getFirstDetectedBallIndex() {
            for (int i = 0; i < colorSensors.length; i++) {
                if (isBallDetected(i)) return i;
            }
            return -1;
        }
    }

    private final ColorSensor[] colorSensors;
    private final SodaColorSensorStrategy strategy;
    private final AutonomousColorSensors auton = new AutonomousColorSensors();

    @Override
    public AutonomousColorSensors getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.processColorData(colorSensors, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        for (int i = 0; i < colorSensors.length; i++) {
            ColorSensor sensor = colorSensors[i];
            telemetry.addData("Sensor " + i,
                    String.format("R:%d G:%d B:%d A:%d",
                            sensor.red(), sensor.green(), sensor.blue(), sensor.alpha()));
        }
    }
}
