package org.firstinspires.ftc.teamcode.opmodes;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "Pedro Pathing Shooter Auto", group = "Autonomous")
public class ShooterAuto extends OpMode {
    private Follower follower;
    private ShooterSubsystem shooter;
    private ElapsedTime stateTimer = new ElapsedTime();

    // 1. Define PoseFactory and field positions (X, Y, Heading in radians/degrees)
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose   = poseFactory.of(12, 60, 0);       // Start position
    private final Pose shootPose   = poseFactory.of(36, 36, 0);       // Shooting position
    private final Pose endPose     = poseFactory.of(12, 12, 0);       // End / Park position


    private enum AutoState {
        DRIVE_TO_SHOOT_POS,
        SPIN_UP,
        SHOOT,
        RESET
    }
    private AutoState currentState = AutoState.DRIVE_TO_SHOOT_POS;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // Build your Pedro paths here...
    }

    @Override
    public void start() {
        stateTimer.reset();
        // follower.followPath(yourShootPath);
    }

    @Override
    public void loop() {
        // Crucial updates for Pedro Pathing and your state machines
        follower.update();
        shooter.update();

        switch (currentState) {
            case DRIVE_TO_SHOOT_POS:
                // Turn on flywheel early while driving to save time
                shooter.startFlywheel();

                if (!follower.isBusy()) { // Path complete
                    currentState = AutoState.SPIN_UP;
                    stateTimer.reset();
                }
                break;

            case SPIN_UP:
                // Ensure the motor has reached full velocity before releasing the indexer
                if (shooter.isFlywheelReady() || stateTimer.seconds() > 1.0) {
                    shooter.triggerIndex(); // Starts the continuous rotation servo
                    currentState = AutoState.SHOOT;
                    stateTimer.reset();
                }
                break;

            case SHOOT:
                // Wait for the shooter subsystem's internal timer to finish indexing
                if (stateTimer.seconds() > 0.6) { // slightly longer than INDEX_DURATION
                    shooter.stopFlywheel();
                    currentState = AutoState.RESET;
                }
                break;

            case RESET:
                // Move to next path or end autonomous
                break;
        }

        // Telemetry feedback
        telemetry.addData("State", currentState);
        telemetry.update();
    }
}

