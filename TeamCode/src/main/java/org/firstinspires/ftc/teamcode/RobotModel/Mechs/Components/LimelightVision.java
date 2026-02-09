package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightVision extends MechComponent {

    public class AutonomousLimelightVision extends AutonomousComponentBehaviors {

        public double getTargetX() {
            // Return X position of detected target
            return 0.0; // Placeholder
        }

        public double getTargetY() {
            // Return Y position of detected target
            return 0.0; // Placeholder
        }

        public boolean hasTarget() {
            // Return true if target is detected
            return false; // Placeholder
        }
    }

    public interface LimelightVisionControlStrategy extends IControlStrategy {
        void processVision(Gamepad gamepad);
    }

    private final AutonomousLimelightVision auton = new AutonomousLimelightVision();
    private final LimelightVisionControlStrategy strategy;
    private final String cameraName;

    public LimelightVision(HardwareMap hardwareMap, String cameraName, LimelightVisionControlStrategy strategy) {
        super(strategy);
        this.cameraName = cameraName;
        this.strategy = strategy;
    }

    @Override
    public AutonomousLimelightVision getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.processVision(gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addData("Limelight Camera", cameraName);
        telemetry.addData("Limelight", "Vision Active");
    }
}
