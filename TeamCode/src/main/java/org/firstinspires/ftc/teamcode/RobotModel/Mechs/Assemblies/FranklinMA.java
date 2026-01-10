package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.FlappyServo;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.NewIntakeServo;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Shooter;

public class FranklinMA extends MechAssembly
{
    public FranklinMA(HardwareMap hardwareMap) {
        intake = new FlappyServo(hardwareMap, "flappyservo", "FLU", "FLD",
                (motor, gamepad) -> {
                    if (gamepad.b)
                        motor.setPower(0.25);
                    else if (gamepad.x) {
                        while (motor.canGoDown())
                            motor.setPower(-0.25);
                        ThreadExtensions.TrySleep(1000);
                        while (motor.canGoUp())
                            motor.setPower(0.25);
                    }
                    else
                        motor.setPower(0);
                });
        canon = new Shooter(hardwareMap, "BANGBANGBANG",
                (motor, gamepad) -> {
                    if (gamepad.right_trigger > 0.01)
                        motor.setPower(gamepad.right_trigger*-1);
                    else if (gamepad.left_trigger > 0.01)
                        motor.setPower(gamepad.left_trigger);
                    else if (gamepad.right_bumper)
                        motor.setPower(-0.72);
                    else
                        motor.setPower(0);
                });;
        auton = new AutonomousFranklinMA(canon.getAutonomousBehaviors(), intake.getAutonomousBehaviors());
        newintake = new NewIntakeServo(hardwareMap, "newintake",
                (motor, gamepad) -> {
                    if (gamepad.y)
                        motor.setPower(0.25);
                    else if (gamepad.a) {
                        motor.setPower(-0.25);
                    }
                    else
                        motor.setPower(0);
                });
    }

    //5 sec spinny spinny

    public class AutonomousFranklinMA extends AutonomousMechBehaviors {
        public final Shooter.AutonomousShooterBehavior AutonShooter;
        public final FlappyServo.AutonomousFlappyBehavior FlappyServo;
        public AutonomousFranklinMA(Shooter.AutonomousShooterBehavior autonShooter, FlappyServo.AutonomousFlappyBehavior flappyServo) {
            this.AutonShooter = autonShooter;
            this.FlappyServo = flappyServo;
        }
    }

    private final AutonomousFranklinMA auton;
    @Override
    public AutonomousFranklinMA getAutonomousBehaviors() { return auton; }

    private final FlappyServo intake;
    private final Shooter canon;
    private final NewIntakeServo newintake;
    @Override
    public void giveInstructions(Gamepad gamepad) {
        intake.move(gamepad);
        canon.move(gamepad);
        newintake.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        intake.update(telemetry);
        canon.update(telemetry);
    }
}
