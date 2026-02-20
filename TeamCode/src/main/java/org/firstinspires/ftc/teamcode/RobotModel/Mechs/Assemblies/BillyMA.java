package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.DoubleShooter;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SpinnyIntake;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.PusherServo;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Turret;

public class BillyMA extends MechAssembly {

    private final SpinnyIntake intake;
    private final PusherServo ballPusher;
    private final DoubleShooter flywheel;
    private final Turret turret;

    public BillyMA(HardwareMap hardwareMap) {
        intake = new SpinnyIntake(hardwareMap, "intakeMotor",
                (motor, gamepad) -> {
                    if (gamepad.a) {

                        motor.setPower(0.75);
                    }
                    if (gamepad.b) {
                        motor.setPower(-0.75);
                    } else {
                        motor.setPower(0);
                    }
                });

        ballPusher = new PusherServo(hardwareMap,
                "ballPusherServo",
                (servoR, gamepad) -> {
                    servoR.setDirection(Servo.Direction.REVERSE);
                    if (gamepad.x) {
                        servoR.setPosition(0.8);
                        //servoL.setPosition(0.22); // Push position (~40 degrees from 0)
                    } else {
                        servoR.setPosition(0.0);
                        //servoL.setPosition(0.0);// Rest position
                    }
                },
                (((servo, telemetry) -> {
                    telemetry.addData("pusher position", servo.getPosition());
                })));

        flywheel = new DoubleShooter(hardwareMap, "flywheelMotorF", "flywheelMotorB",
                (motorF, motorB,gamepad) -> {
                    if (gamepad.y) {
                        motorF.setPower(0.5);
                        motorB.setPower(-0.5);
                    } else {
                        motorF.setPower(0);
                        motorB.setPower(0);
                    }
                });
        // idk how to add nothing for this so i added smth random
        // just don't press the right bumper ig
        turret = new Turret(hardwareMap, "turretServo",
                (servo, gamepad) -> {
//                    if (gamepad.right_bumper)
//                    {
//                        servo.setPower(1);
//                    }
//                    else if (gamepad.left_bumper)
//                    {
//                        servo.setPower(-1);
//                    }
//                    else {
//                        servo.setPower(0);
//                    }
                },
                ((servo, telemetry) -> {
                    telemetry.addData("turret position", servo.getPower());
                }));

        auton = new AutonomousBillyMA(
                intake.getAutonomousBehaviors(),
                ballPusher.getAutonomousBehaviors(),
                flywheel.getAutonomousBehaviors(),
                turret.getAutonomousBehaviors()
        );
    }

    public class AutonomousBillyMA extends AutonomousMechBehaviors {
        public final SpinnyIntake.AutonomousIntakeBehaviors autonIntake;
        public final PusherServo.AutonomousBallPusherBehaviors autonBallPusher;
        public final DoubleShooter.AutonomousShooterBehavior autonFlywheel;
        public final Turret.AutonomousTurretBehaviors autonTurret;

        public AutonomousBillyMA(
                SpinnyIntake.AutonomousIntakeBehaviors autonIntake,
                PusherServo.AutonomousBallPusherBehaviors autonBallPusher,
                DoubleShooter.AutonomousShooterBehavior autonFlywheel,
                Turret.AutonomousTurretBehaviors autonTurret) {
            this.autonIntake = autonIntake;
            this.autonBallPusher = autonBallPusher;
            this.autonFlywheel = autonFlywheel;
            this.autonTurret = autonTurret;
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
        turret.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        ballPusher.update(telemetry);
        flywheel.update(telemetry);
        turret.update(telemetry);
    }
}