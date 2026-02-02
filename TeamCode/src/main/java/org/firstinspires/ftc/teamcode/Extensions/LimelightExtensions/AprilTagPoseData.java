package org.firstinspires.ftc.teamcode.Extensions.LimelightExtensions;

import org.json.JSONObject;

/**
 * Parsed entry from the Limelight "Fiducial" array.
 * <p>
 * Angles are degrees; positions are meters unless noted.
 */
public final class AprilTagPoseData {
    /** Fiducial (AprilTag) ID. */
    public final int fID;
    /** Tag family (16H5C, 25H9C, 36H11C, etc.). */
    public final String fam;
    /** Target area as fraction of image (0..1). */
    public final double ta;
    /** Target x offset in degrees relative to crosshair (right positive). */
    public final double tx;
    /** Target y offset in degrees relative to crosshair (down positive). */
    public final double ty;
    /** Target x offset in degrees relative to principal pixel (right positive). */
    public final double txNoCross;
    /** Target y offset in degrees relative to principal pixel (down positive). */
    public final double tyNoCross;
    /** Target x offset in pixels relative to crosshair (right positive). */
    public final double txp;
    /** Target y offset in pixels relative to crosshair (down positive). */
    public final double typ;
    /** Camera pose in target space (x,y,z,pitch,yaw,roll). */
    public final Pose t6c_ts;
    /** Robot pose in field space (x,y,z,pitch,yaw,roll). */
    public final Pose t6r_fs;
    /** Robot pose in field space from Megatag2 (x,y,z,pitch,yaw,roll). */
    public final Pose t6r_fs_orb;
    /** Robot pose in target space (x,y,z,pitch,yaw,roll). */
    public final Pose t6r_ts;
    /** Target pose in camera space (x,y,z,pitch,yaw,roll). */
    public final Pose t6t_cs;
    /** Target pose in robot space (x,y,z,pitch,yaw,roll). */
    public final Pose t6t_rs;

    private AprilTagPoseData(
            int fID,
            String fam,
            double ta,
            double tx,
            double ty,
            double txNoCross,
            double tyNoCross,
            double txp,
            double typ,
            Pose t6c_ts,
            Pose t6r_fs,
            Pose t6r_fs_orb,
            Pose t6r_ts,
            Pose t6t_cs,
            Pose t6t_rs
    ) {
        this.fID = fID;
        this.fam = fam;
        this.ta = ta;
        this.tx = tx;
        this.ty = ty;
        this.txNoCross = txNoCross;
        this.tyNoCross = tyNoCross;
        this.txp = txp;
        this.typ = typ;
        this.t6c_ts = t6c_ts;
        this.t6r_fs = t6r_fs;
        this.t6r_fs_orb = t6r_fs_orb;
        this.t6r_ts = t6r_ts;
        this.t6t_cs = t6t_cs;
        this.t6t_rs = t6t_rs;
    }

    /**
     * Creates an {@link AprilTagPoseData} from a Limelight fiducial JSON object.
     */
    public static AprilTagPoseData fromJson(JSONObject json) {
        return new AprilTagPoseData(
                json.optInt("fID", -1),
                json.optString("fam", ""),
                json.optDouble("ta", 0.0),
                json.optDouble("tx", 0.0),
                json.optDouble("ty", 0.0),
                json.optDouble("tx_nocross", 0.0),
                json.optDouble("ty_nocross", 0.0),
                json.optDouble("txp", 0.0),
                json.optDouble("typ", 0.0),
                Pose.fromJsonArray(json.optJSONArray("t6c_ts")),
                Pose.fromJsonArray(json.optJSONArray("t6r_fs")),
                Pose.fromJsonArray(json.optJSONArray("t6r_fs_orb")),
                Pose.fromJsonArray(json.optJSONArray("t6r_ts")),
                Pose.fromJsonArray(json.optJSONArray("t6t_cs")),
                Pose.fromJsonArray(json.optJSONArray("t6t_rs"))
        );
    }

    @Override
    public String toString() {
        return "AprilTagPoseData{"
                + "fID=" + fID
                + ", fam='" + fam + '\''
                + ", ta=" + ta
                + ", tx=" + tx
                + ", ty=" + ty
                + ", txNoCross=" + txNoCross
                + ", tyNoCross=" + tyNoCross
                + ", txp=" + txp
                + ", typ=" + typ
                + ", t6c_ts=" + t6c_ts
                + ", t6r_fs=" + t6r_fs
                + ", t6r_fs_orb=" + t6r_fs_orb
                + ", t6r_ts=" + t6r_ts
                + ", t6t_cs=" + t6t_cs
                + ", t6t_rs=" + t6t_rs
                + '}';
    }
}
