package org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions;

import org.json.JSONArray;

/**
 * 6-DOF pose with (x,y,z,pitch,yaw,roll) in meters/degrees as reported by Limelight.
 * <p>
 * The interpretation of the pose (camera in target space, target in robot space, etc.)
 * depends on the field in {@link AprilTagPoseData} that references it.
 */
public final class Pose {
    /** X position in meters. */
    public final double x;
    /** Y position in meters. */
    public final double y;
    /** Z position in meters. */
    public final double z;
    /** Rotation about X axis in degrees. */
    public final double pitch;
    /** Rotation about Y axis in degrees. */
    public final double yaw;
    /** Rotation about Z axis in degrees. */
    public final double roll;

    /**
     * Creates a pose in meters/degrees.
     */
    public Pose(double x, double y, double z, double pitch, double yaw, double roll) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.pitch = pitch;
        this.yaw = yaw;
        this.roll = roll;
    }

    /**
     * Builds a {@link Pose} from a Limelight JSON array of length 6 in the order:
     * x, y, z, pitch, yaw, roll.
     *
     * @return a Pose, or null if the array is missing or too short.
     */
    public static Pose fromJsonArray(JSONArray array) {
        if (array == null || array.length() < 6) {
            return null;
        }
        return new Pose(
                array.optDouble(0, 0.0),
                array.optDouble(1, 0.0),
                array.optDouble(2, 0.0),
                array.optDouble(3, 0.0),
                array.optDouble(4, 0.0),
                array.optDouble(5, 0.0)
        );
    }

    @Override
    public String toString() {
        return "Pose{"
                + "x=" + x
                + ", y=" + y
                + ", z=" + z
                + ", pitch=" + pitch
                + ", yaw=" + yaw
                + ", roll=" + roll
                + '}';
    }
}
