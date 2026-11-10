package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

    public class VisionSubsystem {
    private final Limelight3A limelight;
    private int currentPipeline = 0;

    public static final double TAG_HEIGHT_IN = 0.0; // hive tag center height above the floor
    public static final double CAM_HEIGHT_IN = 0.0; // Limelight lens height above the floor
    public static final double CAM_ANGLE_DEG = 0.0; // lens tilt above horizontal, positive = up


        public VisionSubsystem(HardwareMap hardwareMap) {
        // Initialize based on your Driver Hub configuration name
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

        /** Horizontal distance to the tag in inches, or NaN if no valid target. */
    public double distanceToTagInches() {
            LLResult result = getLatestTarget();
            if (result == null || !result.isValid()) return Double.NaN;

            double angleRad = Math.toRadians(CAM_ANGLE_DEG + result.getTy());
            double tan = Math.tan(angleRad);
            if (Math.abs(tan) < 1e-6) return Double.NaN;

            double d = (TAG_HEIGHT_IN - CAM_HEIGHT_IN) / tan;
            return d > 0 ? d : Double.NaN;
    }
}

