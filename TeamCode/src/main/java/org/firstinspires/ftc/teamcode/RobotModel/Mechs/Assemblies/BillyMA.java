package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutonStrategies.BillyRapidFire;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.DoubleShooter;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SpinnyIntake;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.PusherServo;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Turret;

public class BillyMA extends MechAssembly {

    private final SpinnyIntake intake;
    private final PusherServo ballPusher;
    private final DoubleShooter flywheel;
    private final Turret turret;

    private BillyRapidFire BRF = null;

    private final Telemetry telemetry;

    public BillyMA(HardwareMap hardwareMap, Telemetry tel) {
        intake = new SpinnyIntake(hardwareMap, "intakeMotor",
                (motor, gamepad) -> {
                    if (!BRF.isComplete()) { // checks if BRF is running
                        return; //if it is running, do not proceed
                    }
                    if (gamepad.dpad_up) {

                        motor.setPower(0.75);
                    }
                    if (gamepad.dpad_down) {
                        motor.setPower(-0.75);
                    } else {
                        motor.setPower(0);
                    }
                });

        ballPusher = new PusherServo(hardwareMap,
                "ballPusherServo",
                (servoR, gamepad) -> {
                    if (!BRF.isComplete()) {
                        return;
                    }
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
                    if (!BRF.isComplete()) {
                        return;
                    }
                    double power = 0.45;
                    if (gamepad.dpad_up) {
                        power += 0.05;
                    }
                    if (gamepad.dpad_down) {
                        power -= 0.05;
                    }

                    if (gamepad.y) {
                        motorF.setPower(power);
                        motorB.setPower(-power);
                    } else {
                        motorF.setPower(0);
                        motorB.setPower(0);
                    }
                }, ((motorF, motorB, telemetry) -> {
                    telemetry.addData("power:", motorF.getPower());
        }));
        // idk how to add nothing for this so i added smth random
        // just don't press the right bumper ig
        turret = new Turret(hardwareMap, "turretServo",
                (servo, gamepad) -> {
                    if (!BRF.isComplete()) {
                        return;
                    }
                    if (gamepad.right_bumper)
                    {
                        servo.setPower(-1);
                    }
                    else if (gamepad.left_bumper)
                    {
                        servo.setPower(1);
                    }
                    else {
                        servo.setPower(0);
                    }
                },
                ((servo, telemetry) -> {
                    telemetry.addData("turret position", servo.getPower());
                }));
        this.telemetry = tel;

        auton = new AutonomousBillyMA(
                intake.getAutonomousBehaviors(),
                ballPusher.getAutonomousBehaviors(),
                flywheel.getAutonomousBehaviors(),
                turret.getAutonomousBehaviors()
        );
        BRF = new BillyRapidFire(auton, 3, tel);
        BRF.abort();
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
        if(gamepad.a && BRF.isComplete())
        {
            BRF.reset(3);
            telemetry.addLine("Start Rapid Fire");
            telemetry.update();
        }
        if(!BRF.isComplete())
        {
            BRF.updateState();
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        ballPusher.update(telemetry);
        flywheel.update(telemetry);
        turret.update(telemetry);
    }
}