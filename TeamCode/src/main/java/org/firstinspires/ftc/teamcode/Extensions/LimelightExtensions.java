package org.firstinspires.ftc.teamcode.Extensions;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimelightExtensions {
    /**
     * *look for a specific AprilTag Id using the limelight camera
     * If the tag is not found, this method returns NULL
     * @param limelight camera configured to read Fiducials
     * @param tagId which tag id are you looking for.
     * @return The desired FiducialResult or NULL if not found.
     */

    public static LLResultTypes.FiducialResult tryGetFiducial(
        Limelight3A limelight,
        int tagId)
    {
        return limelight
            .getLatestResult()
            .getFiducialResults()
            .stream()
            .filter(fr -> fr.getFiducialId() == tagId)
            .findFirst()
            .orElse(null);
    }
}