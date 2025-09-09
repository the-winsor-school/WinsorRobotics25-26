package org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Mecanum;

import android.graphics.Path;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Extensions.AngleExtensions;
import org.firstinspires.ftc.teamcode.Extensions.GamepadExtensions;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.Extensions.TurnDirection;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.DriveTrain;

public class MecanumDrive extends DriveTrain
{
    public class AutonomousMecanumDrive extends AutonomousDriving
    {
        // TODO: Write the Autonomous Methods!

        public void turnToAngle(double degrees) {
            degrees = AngleExtensions.mapToIMURange(degrees);
            double yaw = imu.getRobotYawPitchRollAngles().getYaw();
            double smol = AngleExtensions.getSmol(degrees, yaw);
            while(Math.abs(smol) > 1 ) {  //can change dead-zone here
                if (smol > 0) {
                    spin(TurnDirection.LEFT);
                }
                else {
                    spin(TurnDirection.RIGHT);
                }
                ThreadExtensions.TrySleep(100);
                yaw = imu.getRobotYawPitchRollAngles().getYaw();
                smol = AngleExtensions.getSmol(degrees, yaw);

            }
        }

        public void drive(double x, double y, double t)
        {
            double angle = Math.atan2(y, x);
            double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

            double y1 = Math.sin(angle + Math.PI/4) + magnitude;
            double y2 = Math.sin(angle - Math.PI/4) + magnitude;

            double lf = y1 + t;
            double lb = y2 + t;
            double rf = y2 - t;
            double rb = y1 - t;

            double n = Math.max(
                    Math.abs(lf),
                    Math.max(
                            Math.abs(lb),
                            Math.max(
                                    Math.abs(rf),
                                    Math.abs(rb))));

            if (n > 1.0)
            {
                lf/=n;
                lb/=n;
                rf/=n;
                rb/=n;
            }

            RF.setPower(rf);
            RB.setPower(rb);
            LF.setPower(lf);
            LB.setPower(lb);
        }
        public void spin(TurnDirection direction)
        {
            double dir = direction==TurnDirection.RIGHT? 1:-1;
            LB.setPower(dir);
            LF.setPower(dir);
            RB.setPower(-dir);
            RF.setPower(-dir);
        }

    }



    // these can be declared Final because once they are initialized they should not be changed.
    private final DcMotor LB;
    private final DcMotor LF;
    private final DcMotor RB;
    private final DcMotor RF;
    private final IMU imu;

    public static class OrientationConfiguration{
        DcMotorSimple.Direction lb, lf, rb, rf;

        public OrientationConfiguration(
                DcMotorSimple.Direction lb,
                DcMotorSimple.Direction lf,
                DcMotorSimple.Direction rb,
                DcMotorSimple.Direction rf
        )
        {
            this.lb = lb;
            this.lf = lf;
            this.rb = rb;
            this.rf = rf;
        }
        public DcMotorSimple.Direction getLb(){
            return lb;
        }
        public DcMotorSimple.Direction getLf(){
            return lf;
        }
        public DcMotorSimple.Direction getRb(){
            return rb;
        }
        public DcMotorSimple.Direction getRf(){
            return rf;
        }
    }

    public MecanumDrive(
            HardwareMap hardwareMap,
            OrientationConfiguration orientationConfiguration)
    {
        LB = hardwareMap.get(DcMotor.class, "lb");
        LB.setDirection(orientationConfiguration.getLb());
        LF = hardwareMap.get(DcMotor.class, "lf");
        LF.setDirection(orientationConfiguration.getLf());
        RB = hardwareMap.get(DcMotor.class, "rb");
        RB.setDirection(orientationConfiguration.getRb());
        RF = hardwareMap.get(DcMotor.class, "rf");
        RF.setDirection(orientationConfiguration.getRf());

        imu = hardwareMap.get(IMU.class, "imu");

    }

    private final AutonomousMecanumDrive auton = new AutonomousMecanumDrive();

    @Override
    public AutonomousMecanumDrive getAutonomousDriving() {
        return auton;
    }

    @Override
    public void drive(Gamepad gamepad)
    {
        double vertical = GamepadExtensions.GetLeftStickY(gamepad);
        double horizontal = GamepadExtensions.GetLeftStickX(gamepad);
        double turn = GamepadExtensions.GetRightStickX(gamepad);

        double angle = Math.atan2(vertical, horizontal);
        double magnitude = Math.sqrt(Math.pow(horizontal, 2) + Math.pow(vertical, 2));

        double y1 = Math.sin(angle + Math.PI/4) * magnitude;
        double y2 = Math.sin(angle - Math.PI/4) * magnitude;

        double lf = y1 + turn;
        double lb = y2 + turn;
        double rf = y2 - turn;
        double rb = y1 - turn;

        double n = Math.max(
                Math.abs(lf),
                Math.max(
                        Math.abs(lb),
                        Math.max(
                                Math.abs(rf),
                                Math.abs(rb))));

        if (n > 1.0)
        {
            lf/=n;
            lb/=n;
            rf/=n;
            rb/=n;
        }

        RF.setPower(rf);
        RB.setPower(rb);
        LF.setPower(lf);
        LB.setPower(lb);
    }


    /**
     * Don't need anything here yet, might be useful in the future
     * @param telemetry
     */
    @Override
    public void updateTelemetry(Telemetry telemetry)
    {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw();
        telemetry.addData("yaw:", yaw);
    }
}
