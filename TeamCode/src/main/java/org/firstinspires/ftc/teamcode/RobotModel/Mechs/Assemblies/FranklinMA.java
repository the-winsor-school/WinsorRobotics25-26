package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.FlappyServo;
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
                    motor.setPower(gamepad.right_trigger*-1);
                });;
        auton = new AutonomousFranklinMA(canon.getAutonomousBehaviors(), intake.getAutonomousBehaviors());
    }

    //5 sec spinny spinny

    public class AutonomousFranklinMA extends AutonomousMechBehaviors {
        private final Shooter.AutonomousShooterBehavior AutonShooter;
        public final FlappyServo.AutonomousFlappyBehavior FlappyServo;
        public AutonomousFranklinMA(Shooter.AutonomousShooterBehavior autonShooter, FlappyServo.AutonomousFlappyBehavior flappyServo) {
            AutonShooter = autonShooter;
            FlappyServo = flappyServo;
        }
    }

    private final AutonomousFranklinMA auton;
    @Override
    public <T extends AutonomousMechBehaviors> T getAutonomousBehaviors() {
        return null;
    }
    private final FlappyServo intake;
    private final Shooter canon;
    @Override
    public void giveInstructions(Gamepad gamepad) {
        intake.move(gamepad);
        canon.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        intake.update(telemetry);
    }
}
