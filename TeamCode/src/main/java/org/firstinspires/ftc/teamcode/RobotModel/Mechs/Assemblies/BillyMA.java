package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Shooter;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SpinnyIntake;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.PusherServo;

public class BillyMA extends MechAssembly {

    private final SpinnyIntake intake;
    private final PusherServo ballPusher;
    private final Shooter flywheel;

    public BillyMA(HardwareMap hardwareMap) {
        intake = new SpinnyIntake(hardwareMap, "intakeMotor",
                (motor, gamepad) -> {
                    if (gamepad.a) {
                        motor.setPower(0.5);
                    }
                    if (gamepad.b) {
                        motor.setPower(-0.5);
                    } else {
                        motor.setPower(0);
                    }
                });

        ballPusher = new PusherServo(hardwareMap, "ballPusherServo",
                (servo, gamepad) -> {
                    if (gamepad.x) {
                        servo.setPosition(0.22);  // Push position (~40 degrees from 0)
                    } else {
                        servo.setPosition(0);   // Rest position
                    }
                });

        flywheel = new Shooter(hardwareMap, "flywheelMotor",
                (motor, gamepad) -> {
                    if (gamepad.y) {
                        motor.setPower(-0.75);
                    } else {
                        motor.setPower(0);
                    }
                });

        auton = new AutonomousBillyMA(
                intake.getAutonomousBehaviors(),
                ballPusher.getAutonomousBehaviors(),
                flywheel.getAutonomousBehaviors()
        );
    }

    public class AutonomousBillyMA extends AutonomousMechBehaviors {
        public final SpinnyIntake.AutonomousIntakeBehaviors autonIntake;
        public final PusherServo.AutonomousBallPusherBehaviors autonBallPusher;
        public final Shooter.AutonomousShooterBehavior autonFlywheel;

        public AutonomousBillyMA(
                SpinnyIntake.AutonomousIntakeBehaviors autonIntake,
                PusherServo.AutonomousBallPusherBehaviors autonBallPusher,
                Shooter.AutonomousShooterBehavior autonFlywheel) {
            this.autonIntake = autonIntake;
            this.autonBallPusher = autonBallPusher;
            this.autonFlywheel = autonFlywheel;
        }
    }

    private final AutonomousBillyMA auton;

    @Override
    public AutonomousBillyMA getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void giveInstructions(Gamepad gamepad) {
        intake.move(gamepad);
        ballPusher.move(gamepad);
        flywheel.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        ballPusher.update(telemetry);
        flywheel.update(telemetry);
    }
}