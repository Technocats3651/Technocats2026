package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem {

    // Hive AprilTag IDs (Competition Manual, Arena section)
    public static final int[] RED_SCORING_CELL_TAGS   = {30, 31, 32, 33};
    public static final int[] RED_AUDIENCE_CELL_TAGS  = {34, 35, 36, 37};
    public static final int[] BLUE_AUDIENCE_CELL_TAGS = {38, 39, 40, 41};
    public static final int[] BLUE_SCORING_CELL_TAGS  = {42, 43, 44, 45};

    // All tags for each alliance, used when either cell is acceptable
    public static final int[] RED_HIVE_TAGS  = {30, 31, 32, 33, 34, 35, 36, 37};
    public static final int[] BLUE_HIVE_TAGS = {38, 39, 40, 41, 42, 43, 44, 45};

    // Camera geometry in inches and degrees. Measure these before use.
    // While they are 0.0, distanceToTagsInches returns NaN and callers fall back to odometry.
    public static final double TAG_HEIGHT_IN = 50.0; // tag center height above the floor
    public static final double CAM_HEIGHT_IN = 0.0; // lens height above the floor
    public static final double CAM_ANGLE_DEG = 45.0; // lens tilt above horizontal, positive = up

    private final Limelight3A limelight;
    private int currentPipeline = 0;

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline);
    }

    public void startCamera() {
        limelight.start();
    }

    public void switchPipeline(int pipelineId) {
        this.currentPipeline = pipelineId;
        limelight.pipelineSwitch(pipelineId);
    }

    public LLResult getLatestTarget() {
        return limelight.getLatestResult();
    }

    public boolean hasTarget() {
        LLResult result = getLatestTarget();
        return result != null && result.isValid();
    }

    /** Detected tags whose ID is in ids. Empty list if none. */
    public List<LLResultTypes.FiducialResult> getTagsIn(int[] ids) {
        List<LLResultTypes.FiducialResult> matched = new ArrayList<>();
        LLResult result = getLatestTarget();
        if (result == null || !result.isValid()) return matched;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            for (int id : ids) {
                if (tag.getFiducialId() == id) {
                    matched.add(tag);
                    break;
                }
            }
        }
        return matched;
    }

    public boolean hasTagIn(int[] ids) {
        return !getTagsIn(ids).isEmpty();
    }

    /** Mean horizontal angle in degrees of the matched tags, or NaN. */
    public double averageTx(int[] ids) {
        List<LLResultTypes.FiducialResult> tags = getTagsIn(ids);
        if (tags.isEmpty()) return Double.NaN;
        double sum = 0;
        for (LLResultTypes.FiducialResult t : tags) sum += t.getTargetXDegrees();
        return sum / tags.size();
    }

    /** Mean vertical angle in degrees of the matched tags, or NaN. */
    public double averageTy(int[] ids) {
        List<LLResultTypes.FiducialResult> tags = getTagsIn(ids);
        if (tags.isEmpty()) return Double.NaN;
        double sum = 0;
        for (LLResultTypes.FiducialResult t : tags) sum += t.getTargetYDegrees();
        return sum / tags.size();
    }

    /** Horizontal distance in inches to the mean of the matched tags, or NaN. */
    public double distanceToTagsInches(int[] ids) {
        double ty = averageTy(ids);
        if (Double.isNaN(ty)) return Double.NaN;

        double tan = Math.tan(Math.toRadians(CAM_ANGLE_DEG + ty));
        if (Math.abs(tan) < 1e-6) return Double.NaN;

        double d = (TAG_HEIGHT_IN - CAM_HEIGHT_IN) / tan;
        return d > 0 ? d : Double.NaN;
    }
}
