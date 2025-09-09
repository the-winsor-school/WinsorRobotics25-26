package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.GamepadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Claw;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.DoublyLimitedMotor;

public class CascadeArm extends MechAssembly
{
    public class AutonomousCascadeArm extends AutonomousMechBehaviors
    {
        public final DoublyLimitedMotor.AutonomousDLMBehaviors cascade;
        public final DoublyLimitedMotor.AutonomousDLMBehaviors drawbridge;
        public final Claw.AutonomousClawBehaviors claw;

        public AutonomousCascadeArm(
                DoublyLimitedMotor.AutonomousDLMBehaviors cascade,
                DoublyLimitedMotor.AutonomousDLMBehaviors drawbridge,
                Claw.AutonomousClawBehaviors claw)
        {
            this.cascade = cascade;
            this.drawbridge = drawbridge;
            this.claw = claw;
        }
    }

    private final AutonomousCascadeArm auton;

    @Override
    public AutonomousCascadeArm getAutonomousBehaviors()
    {
        return auton;
    }

    private final DoublyLimitedMotor cascade;
    private final DoublyLimitedMotor drawbridge;
    private final Claw claw;
    public CascadeArm (HardwareMap hardwareMap){
        cascade = new DoublyLimitedMotor(
                hardwareMap,
                "cascadeMotor",
                "topLiftLimit",
                "bottomLiftLimit",
                (gamepad, dlm) ->
                {
                 double power = GamepadExtensions.GetLeftStickY(gamepad);
                 dlm.setPower(power);
                },
                ((dlm, telemetry) ->
                {
                    telemetry.addData("Can Go Up", dlm.canGoForward());
                    telemetry.addData("Can Go Down", dlm.canGoReverse());
                })
                );
        drawbridge = new DoublyLimitedMotor(
                hardwareMap,
                "drawbridge",
                "topDrawLimit",
                "botDrawLimit",
                (gamepad, dlm) -> {
                    double power = GamepadExtensions.GetRightStickY(gamepad);
                    dlm.setPower(power);
                },
                ((dlm, telemetry) -> {
                    telemetry.addData("Can Go Up", dlm.canGoForward());
                    telemetry.addData("Can Go Down", dlm.canGoReverse());
                }));
        claw = new Claw(
                hardwareMap,
                "servo",
                (servo, gamepad) -> {
                    if (gamepad.a) servo.setPosition(1.0);
                    if (gamepad.b) servo.setPosition(0.0);
                },
                (servo, telemetry) -> {
                    telemetry.addData("Claw Position", servo.getPosition());
                }
        );

        auton = new AutonomousCascadeArm(
                cascade.getAutonomousBehaviors(),
                drawbridge.getAutonomousBehaviors(),
                claw.getAutonomousBehaviors());
    }

    @Override
    public void giveInstructions(Gamepad gamepad) {
        cascade.move(gamepad);
        drawbridge.move(gamepad);
        claw.move(gamepad);
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        claw.update(telemetry);
        cascade.update(telemetry);
        drawbridge.update(telemetry);
    }
}
