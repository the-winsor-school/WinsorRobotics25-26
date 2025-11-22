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
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import java.util.List;

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

        /**
         * TODO:  Comment your methods!
         * TODO:  Why do we pass two color processors and then only use one?
         *        Only pass one color processor
         *        (and pass in the one that is for the color you want to search for)
         *        This will make your code much more robust and will work for future seasons
         * @param greenProcessor
         * @param purpleProcessor
         * @param targetColor
         */
        public void LocateAndDriveToColor(ColorBlobLocatorProcessor greenProcessor,
                                          ColorBlobLocatorProcessor purpleProcessor,
                                          String targetColor) {
            // Constants for navigation control
            // TODO:  Another way to make this method more robust would be to pass all of these
            //        variables as parameters.  Bonus points if you make this into an OBJECT as well
            //        Something like a "ColorSearchConfiguration" perhaps?  (similar to OrientationConfiguration)
            final double TARGET_AREA_THRESHOLD = 5000.0;
            final double CENTER_TOLERANCE = 50.0;
            final double APPROACH_SPEED = 0.3;
            final double TURN_SPEED = 0.2;
            final int MAX_SEARCH_TIME = 5000;
            final int CAMERA_CENTER_X = 320;

            ColorBlobLocatorProcessor activeProcessor;
            if (targetColor.equalsIgnoreCase("green")) {
                activeProcessor = greenProcessor;
            } else if (targetColor.equalsIgnoreCase("purple")) {
                activeProcessor = purpleProcessor;
            } else {
                stop();
                return;
            }

            int searchTime = 0;
            boolean targetFound = false;

            //search for the specified color blob
            // TODO:  searchTime isn't really measuring time!~
            //        also, targetFound is not actually used here.
            while (!targetFound && searchTime < MAX_SEARCH_TIME) {
                List<ColorBlobLocatorProcessor.Blob> blobs = activeProcessor.getBlobs();

                if (!blobs.isEmpty()) {
                    targetFound = true;
                    break;
                }

                spin(TurnDirection.LEFT);
                ThreadExtensions.TrySleep(100);
                stop();
                ThreadExtensions.TrySleep(50);

                searchTime += 150;
            }

            // TODO:  Implementing a "State Machine" would negate the need for a lot of loops
            //        and If Statements here~
            if (!targetFound) {
                stop();
                return;
            }

            //navigate to the largest color blob of specified color
            boolean targetReached = false;
            int navigationTime = 0;
            final int MAX_NAVIGATION_TIME = 10000;
            // TODO: break this down into helper methods~  Those helper methods are the seeds for
            //       different "States" for the State Machine
            while (!targetReached && navigationTime < MAX_NAVIGATION_TIME) {
                List<ColorBlobLocatorProcessor.Blob> blobs = activeProcessor.getBlobs();

                if (blobs.isEmpty()) {
                    stop();
                    ThreadExtensions.TrySleep(200);

                    spin(TurnDirection.LEFT);
                    ThreadExtensions.TrySleep(300);
                    stop();

                    navigationTime += 500;
                    continue;
                }

                ColorBlobLocatorProcessor.Blob targetBlob = blobs.get(0);
                double blobCenterX = targetBlob.getBoxFit().center.x;
                double blobArea = targetBlob.getContourArea();

                if (blobArea > TARGET_AREA_THRESHOLD) {
                    stop();
                    targetReached = true;
                    break;
                }

                double horizontalOffset = blobCenterX - CAMERA_CENTER_X;

                if (Math.abs(horizontalOffset) > CENTER_TOLERANCE) {
                    if (horizontalOffset > 0) {
                        drive(0, 0, TURN_SPEED);
                    } else {
                        drive(0, 0, -TURN_SPEED);
                    }
                } else {
                    drive(0, APPROACH_SPEED, 0);
                }

                ThreadExtensions.TrySleep(50);
                navigationTime += 50;
            }
            stop();
        }

        /**
         * TODO:  Comment your Methods!
         * @param greenProcessor
         * @param purpleProcessor
         */
        public void LocateAndDriveToGreen(ColorBlobLocatorProcessor greenProcessor,
                                          ColorBlobLocatorProcessor purpleProcessor) {
            LocateAndDriveToColor(greenProcessor, purpleProcessor, "green");
        }

        /**
         * TODO:  Comment your Methods!
         * @param greenProcessor
         * @param purpleProcessor
         */
        public void LocateAndDriveToPurple(ColorBlobLocatorProcessor greenProcessor,
                                           ColorBlobLocatorProcessor purpleProcessor) {
            LocateAndDriveToColor(greenProcessor, purpleProcessor, "purple");
        }

        public void stop() {
            RF.setPower(0);
            RB.setPower(0);
            LF.setPower(0);
            LB.setPower(0);
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
