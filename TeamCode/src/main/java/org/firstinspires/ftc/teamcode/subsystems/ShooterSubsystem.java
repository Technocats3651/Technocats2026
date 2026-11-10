package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSubsystem {
    // Hardware Components
    private DcMotorEx flywheel;
    private CRServo indexer;

    // Subsystem States
    public enum IndexerState {
        IDLE,
        INDEXING
    }
    private IndexerState indexerState = IndexerState.IDLE;

    // Timing constants
    private ElapsedTime indexTimer = new ElapsedTime();
    private static final double INDEX_DURATION = 0.5; // Seconds it takes to index one game piece
    private static final double INDEXER_POWER = 1.0;
    private static final double INDEXER_STOP = 0.0;

    // Shooter Speeds
    public static double SHOOT_VELOCITY = 2000; // Encoder ticks per second (adjust to your needs)

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        indexer = hardwareMap.get(CRServo.class, "Indexer");

        // Optimize flywheel behavior
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    // --- Flywheel Controls ---
    public void startFlywheel() {
        flywheel.setVelocity(SHOOT_VELOCITY);
    }

    public void stopFlywheel() {
        flywheel.setVelocity(0);
    }

    public boolean isFlywheelReady() {
        // Returns true if flywheel is within 50 ticks/sec of target speed
        return Math.abs(flywheel.getVelocity() - SHOOT_VELOCITY) < 50;
    }

    // --- Indexer Controls ---
    public void triggerIndex() {
        // Only start indexing if we aren't already indexing
        if (indexerState == IndexerState.IDLE) {
            indexerState = IndexerState.INDEXING;
            indexTimer.reset();
        }
    }

    /**
     * This method MUST be called continuously in your OpMode loop.
     * It handles the non-blocking timing to stop the CR servo.
     */
    public void update() {
        switch (indexerState) {
            case IDLE:
                indexer.setPower(INDEXER_STOP);
                break;

            case INDEXING:
                if (indexTimer.seconds() < INDEX_DURATION) {
                    indexer.setPower(INDEXER_POWER);
                } else {
                    indexer.setPower(INDEXER_STOP);
                    indexerState = IndexerState.IDLE;
                }
                break;
        }
    }
}

