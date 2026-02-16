/* RobotModel/Odometry/OdometryTracker.java */
package org.firstinspires.ftc.teamcode.RobotModel.Odometry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdometryTracker {

    private final DcMotorEx leftEncoder;
    private final DcMotorEx rightEncoder;
    private final DcMotorEx strafeEncoder;

    // Robot dimensions (in inches)
    private static final double TICKS_PER_INCH = 537.7 / (3.78 * Math.PI);  // Adjust based on your wheel
    private static final double TRACK_WIDTH = 15.0;  // Distance between left and right encoders (inches)
    private static final double STRAFE_OFFSET = 8.0;  // Distance from center to strafe encoder (inches)

    // Position tracking
    private double x = 0;
    private double y = 0;
    private double heading = 0;  // in radians

    // Previous encoder values
    private long prevLeftTicks = 0;
    private long prevRightTicks = 0;
    private long prevStrafeTicks = 0;

    /**
     * Initialize odometry with three encoders
     * @param hardwareMap HardwareMap from OpMode
     * @param leftMotorName Name of left drive motor (has encoder)
     * @param rightMotorName Name of right drive motor (has encoder)
     * @param strafeMotorName Name of strafe motor (has encoder)
     */
    public OdometryTracker(HardwareMap hardwareMap,
                           String leftMotorName,
                           String rightMotorName,
                           String strafeMotorName) {
        this.leftEncoder = hardwareMap.get(DcMotorEx.class, leftMotorName);
        this.rightEncoder = hardwareMap.get(DcMotorEx.class, rightMotorName);
        this.strafeEncoder = hardwareMap.get(DcMotorEx.class, strafeMotorName);

        resetEncoders();
    }

    /**
     * Reset all encoders to zero
     */
    public void resetEncoders() {
        leftEncoder.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeEncoder.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prevLeftTicks = 0;
        prevRightTicks = 0;
        prevStrafeTicks = 0;
    }

    /**
     * Reset position to (0, 0) with heading 0
     */
    public void resetPosition() {
        x = 0;
        y = 0;
        heading = 0;
        resetEncoders();
    }

    /**
     * Update odometry based on encoder readings
     * Call this frequently (every loop iteration)
     */
    public void update() {
        long leftTicks = leftEncoder.getCurrentPosition();
        long rightTicks = rightEncoder.getCurrentPosition();
        long strafeTicks = strafeEncoder.getCurrentPosition();

        // Calculate deltas
        long deltaLeft = leftTicks - prevLeftTicks;
        long deltaRight = rightTicks - prevRightTicks;
        long deltaStrafe = strafeTicks - prevStrafeTicks;

        // Convert ticks to inches
        double deltaLeftInches = deltaLeft / TICKS_PER_INCH;
        double deltaRightInches = deltaRight / TICKS_PER_INCH;
        double deltaStrafeInches = deltaStrafe / TICKS_PER_INCH;

        // Calculate change in heading (radians)
        double deltaHeading = (deltaLeftInches - deltaRightInches) / TRACK_WIDTH;

        // Calculate forward and strafe movement
        double deltaForward = (deltaLeftInches + deltaRightInches) / 2.0;
        double deltaX = deltaStrafeInches - (deltaHeading * STRAFE_OFFSET);

        // Update heading
        heading += deltaHeading;

        // Update position
        x += deltaForward * Math.cos(heading) - deltaX * Math.sin(heading);
        y += deltaForward * Math.sin(heading) + deltaX * Math.cos(heading);

        // Update previous values
        prevLeftTicks = leftTicks;
        prevRightTicks = rightTicks;
        prevStrafeTicks = strafeTicks;
    }

    /**
     * Get current X position (inches)
     */
    public double getX() {
        return x;
    }

    /**
     * Get current Y position (inches)
     */
    public double getY() {
        return y;
    }

    /**
     * Get current heading (radians)
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Get current heading in degrees
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }

    /**
     * Set position manually (useful for reset)
     */
    public void setPosition(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Update telemetry with odometry data
     */
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine("===== ODOMETRY =====");
        telemetry.addData("X Position (in)", String.format("%.2f", x));
        telemetry.addData("Y Position (in)", String.format("%.2f", y));
        telemetry.addData("Heading (degrees)", String.format("%.2f", getHeadingDegrees()));
        telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
        telemetry.addData("Strafe Encoder", strafeEncoder.getCurrentPosition());
        telemetry.addLine("====================");
    }
}

