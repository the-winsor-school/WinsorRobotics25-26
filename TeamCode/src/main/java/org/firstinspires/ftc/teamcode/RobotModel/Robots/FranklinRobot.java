package org.firstinspires.ftc.teamcode.RobotModel.Robots;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.DriveTrain.Tank.StandardTankDrive;
import org.firstinspires.ftc.teamcode.RobotModel.Mechs.Assemblies.FranklinMA;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class FranklinRobot extends Robot {
    public class AutonomousFranklinRobot extends AutonomousRobot {
        public final StandardTankDrive.AutonomousTankDrive driveTrain;
        public final FranklinMA.AutonomousFranklinMA mechAssembly;

        public AutonomousFranklinRobot(
                StandardTankDrive.AutonomousTankDrive driveTrain,
                FranklinMA.AutonomousFranklinMA mechAssembly)
        {
            super(driveTrain, mechAssembly);
            this.driveTrain = driveTrain;
            this.mechAssembly = mechAssembly;
        }
        /**
         * Locates the red goal AprilTag and returns pose information
         * @param targetTagId The ID of the red goal AprilTag (you'll need to know this for
         * @return AprilTagDetection if found, null if not found
         */
        public AprilTagDetection locateRedGoalAprilTag(int targetTagId) {
            List <AprilTagDetection> detections = getAprilTagDetections();

            for(AprilTagDetection detection : detections) {
                if (detection.id == targetTagId) {
                    return detection; //Found the red goal tag!
                }
            }
            return null; //Tag not found
        }
        public boolean driveToRedGoal (int targetTagId, double targetDistance) {
            AprilTagDetection detection = locateRedGoalAprilTag(targetTagId);

            if (detection == null){
                return false; //Can't find the tag
            }
            //Get pose information (distance and bearing from camera)
            double currentDistance = detection.ftcPose.range;
            double bearing = detection.ftcPose.bearing; //Horizontal angle to tag

            //Simple navigation logic for tank drive
            if (Math.abs(currentDistance - targetDistance )> 2.0) { //2 inch tolerance
                if (currentDistance > targetDistance) {
                    //Too far away, drive forward
                    driveTrain.driveForward();
                    ThreadExtensions.TrySleep(200);
                    driveTrain.stop();
                } else {
                    //Too close, drive backward
                    driveTrain.driveBackward();
                    ThreadExtensions.TrySleep(200);
                    driveTrain.stop();
                }
            }
            //Adjust heading if needed (bearing in degrees)
            if (Math.abs(bearing) > 5.0) {
                if (bearing>0){
                    //Tag is to the right, turn right
                    driveTrain.turnRight();
                    ThreadExtensions.TrySleep(100);
                    driveTrain.stop();
                } else {
                    //Tag is to the left, turn left
                    driveTrain.turnLeft();
                    ThreadExtensions.TrySleep(100);
                    driveTrain.stop();
                }
            }
            return true; //
        }

    }
        private final AutonomousFranklinRobot auton;

    @Override
    public AutonomousFranklinRobot getAutonomousRobot() {

        return auton;
    }

    private final AprilTagProcessor aprilTagProcessor;
    private final ColorBlobLocatorProcessor purpleBallProcessor;
    private final ColorBlobLocatorProcessor greenBallProcessor;

    public FranklinRobot(HardwareMap hardwareMap) {
        driveTrain = new StandardTankDrive(hardwareMap);

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        purpleBallProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .build();

        greenBallProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(purpleBallProcessor)
                .addProcessor(greenBallProcessor)
                .addProcessor(aprilTagProcessor)
                // this is supposed to make it show up on the driverstation....
                // it works, but you can only do it at a very specific time...
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
        mechAssembly = new FranklinMA(hardwareMap);
        auton = new AutonomousFranklinRobot(driveTrain.getAutonomousDriving(), mechAssembly.getAutonomousBehaviors());
    }
    /**
     * * Get current AprilTag detections
     * @return List of detected AprilTags
     */
    public List<AprilTagDetection> getAprilTagDetections(){
        return aprilTagProcessor.getDetections();
    }
}