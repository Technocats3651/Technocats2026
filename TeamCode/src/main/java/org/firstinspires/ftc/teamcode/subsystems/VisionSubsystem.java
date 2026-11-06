package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

    public class VisionSubsystem {
    private final Limelight3A limelight;
    private int currentPipeline = 0;

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
}

