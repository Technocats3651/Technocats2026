package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;


// Ivy command imports
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;
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
        DRIVE_TO_END_POS,
        RESET
    }
    private AutoState currentState = AutoState.DRIVE_TO_SHOOT_POS;

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // Set the robot's initial pose on the field
        follower.setPose(startPose);
    }

    // Build paths cleanly using Paths methods
    private Path startToShootPath() {
        return Paths.line(startPose, shootPose).linear(startPose, shootPose);
    }

    private Path shootToEndPath() {
        return Paths.line(shootPose, endPose).linear(shootPose, endPose);
    }

    @Override
    public void start() {
        stateTimer.reset();
        follower.follow(startToShootPath());

    }

    @Override
    public void loop() {
        // Crucial updates for Pedro Pathing and your state machines
        follower.update();
        shooter.update();
        Scheduler.execute();

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
                // Wait a brief moment to finish shooting, then drive to end/park
                if (stateTimer.seconds() > 1.5) {
                    shooter.stopFlywheel();
                    follower.follow(shootToEndPath()); // Hold at end pose
                    currentState = AutoState.DRIVE_TO_END_POS;
                }
                break;

            case DRIVE_TO_END_POS:
                if (!follower.isBusy()) {
                    currentState = AutoState.RESET;
                }
                break;

            case RESET:
                // Autonomous finished
                break;
        }
    }
}


