package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum.MecanumDrive;

public class AppleRobot extends Robot {
    @Override
    public <T extends AutonomousRobot> T getAutonomousRobot() {
        return null;
    }

    public AppleRobot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        super(telemetry);
        driveTrain = new MecanumDrive(
            hardwareMap, //step up mec drive- wheels mec
            new MecanumDrive.OrientationConfiguration(
                    DcMotorSimple.Direction.REVERSE,
                    DcMotorSimple.Direction.FORWARD,
                    DcMotorSimple.Direction.FORWARD,
                    DcMotorSimple.Direction.REVERSE

            /* next step: create mec-:
                spinny intake and spinny outake- (shooter)
                2 motors, 1 servo(?)
            */
            )
        );
        initializeSubsystems();
    }
}
