package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.Shooter;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components.SpinnyIntake;

public class FranklinMA extends MechAssembly{
    public FranklinMA(HardwareMap hardwareMap) {
        intake = new SpinnyIntake(hardwareMap, "THENOMS",
                (motor, gamepad) -> {
                    if(gamepad.a)
                        motor.setPower(1);
                    else if(gamepad.b)
                        motor.setPower(-1);
                    else
                        motor.setPower(0);
                });
        canon = new Shooter(hardwareMap, "BANGBANGBANG",
                (motor, gamepad) -> {
                    motor.setPower(gamepad.right_trigger);
                });;
    }

    @Override
    public <T extends AutonomousMechBehaviors> T getAutonomousBehaviors() {
        return null;
    }
    private final SpinnyIntake intake;

    private final Shooter canon;
    @Override
    public void giveInstructions(Gamepad gamepad) {

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
