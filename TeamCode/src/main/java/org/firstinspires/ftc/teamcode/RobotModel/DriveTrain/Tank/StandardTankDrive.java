package org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Tank;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.GamepadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.DriveTrain;

public class StandardTankDrive extends DriveTrain
{
    public class AutonomousTankDrive extends AutonomousDriving
    {

    }

    private final AutonomousTankDrive auton = new AutonomousTankDrive();

    @Override
    public AutonomousTankDrive getAutonomousDriving()
    {
        return auton;
    }

    // these can be declared Final because once they are initialized they should not be changed.
    private final DcMotor left, right;

    /**
     * Initialize the Tank Drive with Configuration motors named:
     * left -> "leftTread"
     * right -> "rightTread"
     * @param hardwareMap pass down from OpMode.
     */
    public StandardTankDrive(HardwareMap hardwareMap)
    {
        left = hardwareMap.get(DcMotor.class, "leftTread");
        right = hardwareMap.get(DcMotor.class, "rightTread");
    }

    @Override
    public void drive(Gamepad gamepad)
    {
        float leftDrive = GamepadExtensions.GetLeftStickY(gamepad);
        float rightDrive = GamepadExtensions.GetRightStickY(gamepad);
        left.setPower(leftDrive);
        right.setPower(rightDrive);
    }

    /**
     * Don't need anything here yet, might be useful in the future
     * @param telemetry
     */
    @Override
    public void updateTelemetry(Telemetry telemetry)
    {

    }

}
