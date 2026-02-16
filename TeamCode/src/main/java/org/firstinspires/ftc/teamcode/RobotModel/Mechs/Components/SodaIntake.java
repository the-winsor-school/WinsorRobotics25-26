/* RobotModel/Mechs/Components/SodaIntake.java */
package org.firstinspires.ftc.teamcode.RobotModel.Mechs.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SodaIntake extends MechComponent {

    public interface SodaIntakeControlStrategy extends IControlStrategy {
        void controlIntake(DcMotor motor, Gamepad gamepad);
    }

    public SodaIntake(HardwareMap hardwareMap,
                      String motorName,
                      SodaIntakeControlStrategy strategy) {
        super(strategy);
        this.intakeMotor = hardwareMap.get(DcMotor.class, motorName);
        this.strategy = strategy;
    }

    public class AutonomousSodaIntake extends AutonomousComponentBehaviors {

        public void intakeIn() {
            intakeMotor.setPower(1.0);
        }

        public void intakeOut() {
            intakeMotor.setPower(-1.0);
        }

        public void stopIntake() {
            intakeMotor.setPower(0);
        }
    }

    private final DcMotor intakeMotor;
    private final SodaIntakeControlStrategy strategy;
    private final AutonomousSodaIntake auton = new AutonomousSodaIntake();

    @Override
    public AutonomousSodaIntake getAutonomousBehaviors() {
        return auton;
    }

    @Override
    public void move(Gamepad gamepad) {
        strategy.controlIntake(intakeMotor, gamepad);
    }

    @Override
    public void update(Telemetry telemetry) {
        telemetry.addData("Intake Power", String.format("%.2f", intakeMotor.getPower()));
    }
}
