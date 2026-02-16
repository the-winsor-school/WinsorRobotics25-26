package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.AimingAssistant;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.BallDetectionComponent;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.LimelightVision;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaFlywheel;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaIntake;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaLift;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SodaSpindexServo;

public class SodapopMA extends MechAssembly {

    public SodapopMA(HardwareMap hardwareMap) {
        sodamouth = new SodaIntake(hardwareMap, "soinmotor",
                (motor, gamepad) -> {
                    if (gamepad.right_trigger != 0) {
                        motor.setPower(gamepad.right_trigger);
                    } else {
                        motor.setPower(gamepad.left_trigger);
                    }
                });

        sodaflywheel = new SodaFlywheel(hardwareMap, "soflymotor",
                (motor, gamepad) -> {
                    if (gamepad.right_bumper) {
                        motor.setPower(0.75);
                    } else if (gamepad.left_bumper) {
                        motor.setPower(-1);
                    }
                });

        sodaspindex = new SodaSpindexServo(hardwareMap, "sospinservo",
                (servo, gamepad) -> {
                    if (gamepad.x) {
                        ((SodaSpindexServo.AutonomousSpindy)
                                SodapopMA.this.sodaspindex.getAutonomousBehaviors()).rotateTo120Degrees();
                    }
                });

        limelightVision = new LimelightVision(hardwareMap, "limelight",
                gamepad -> {
                    // Limelight reads data passively
                });

        // NEW: Initialize Aiming Assistant
        aimingAssistant = new AimingAssistant();

        auton = new AutonomousSodapopMA(
                sodamouth.getAutonomousBehaviors(),
                sodaflywheel.getAutonomousBehaviors(),
                sodaspindex.getAutonomousBehaviors(),
                limelightVision.getAutonomousBehaviors()
        );

        balldetector = new BallDetectionComponent(hardwareMap, "Webcam 1",
                (purpleProcessor, greenProcessor, gamepad) -> {});
    }

    public class AutonomousSodapopMA extends AutonomousMechBehaviors {
        private final SodaIntake.AutonomousSodaIntake AutonSOIN;
        private final SodaFlywheel.AutonomousSoFly AutonSOFLY;
        private final SodaSpindexServo.AutonomousSpindy AutonSOSS;
        private final LimelightVision.AutonomousLimelightVision AutonLIMELIGHT;

        public AutonomousSodapopMA(
                SodaIntake.AutonomousSodaIntake autonSOIN,
                SodaFlywheel.AutonomousSoFly autonSOFLY,
                SodaSpindexServo.AutonomousSpindy autonSOSS,
                LimelightVision.AutonomousLimelightVision autonLIMELIGHT) {
            AutonSOIN = autonSOIN;
            AutonSOFLY = autonSOFLY;
            AutonSOSS = autonSOSS;
            AutonLIMELIGHT = autonLIMELIGHT;
        }
    }

    private final AutonomousSodapopMA auton;

    @Override
    public AutonomousSodapopMA getAutonomousBehaviors() {
        return auton;
    }

    private final SodaIntake sodamouth;
    private final SodaFlywheel sodaflywheel;
    private final SodaSpindexServo sodaspindex;
    private final LimelightVision limelightVision;
    private final BallDetectionComponent balldetector;
    private final AimingAssistant aimingAssistant;  // NEW

    @Override
    public void giveInstructions(Gamepad gamepad) {
        // Update aiming mode based on gamepad input
        aimingAssistant.updateAimingMode(gamepad);

        sodamouth.move(gamepad);
        sodaflywheel.move(gamepad);
        sodaspindex.move(gamepad);
        limelightVision.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        limelightVision.update(telemetry);
        balldetector.update(telemetry);
    }

    // NEW: Getter for aiming assistant
    public AimingAssistant getAimingAssistant() {
        return aimingAssistant;
    }

    // NEW: Getter for limelight behaviors (used in auton)
    public LimelightVision.AutonomousLimelightVision getLimelightBehaviors() {
        return auton.AutonLIMELIGHT;
    }
}
