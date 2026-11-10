package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage; // 👈 Your OpModeStorage import
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterIndexerSubsystem;

@Autonomous(name = "Ivy Pedro Sequential Shooter Auto V2", group = "Autonomous")
public class PedroIvyLimelightAutoV2 extends OpMode {

    private Follower follower;
    private VisionSubsystem vision;
    private IntakeSubsystem intake;
    private ShooterIndexerSubsystem shooter;

    private Path absolutePreDrivePath;
    private Path alignmentPath;
    private Path parkPath;

    // --- Alliance Selection States ---
    private enum Alliance { RED, BLUE }
    private Alliance currentAlliance = Alliance.RED;
    private boolean lastGamepad1X = false;
    private boolean lastGamepad1B = false;

    // --- Core Master Coordinates (Always Defined from Red Perspective) ---
    private final Pose redStartPose = new Pose(24, 24, Math.toRadians(0));
    private final Pose redTargetAreaPose = new Pose(60, 36, Math.toRadians(45));
    private final Pose redLoadingZoneParkPose = new Pose(12, 12, Math.toRadians(90)); // Example Red park point

    private Pose startPose;
    private Pose targetAreaPose;
    private Pose loadingZoneParkPose;
    private long alignStartMs = 0;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        shooter = new ShooterIndexerSubsystem(hardwareMap);
        vision.switchPipeline(0);

        telemetry.addData("Status", "Initialized. Select Alliance on Gamepad1.");
    }

    @Override
    public void init_loop() {
        if (gamepad1.b && !lastGamepad1B) currentAlliance = Alliance.RED;
        if (gamepad1.x && !lastGamepad1X) currentAlliance = Alliance.BLUE;
        lastGamepad1B = gamepad1.b;
        lastGamepad1X = gamepad1.x;

        telemetry.addLine("=== ALLIANCE CONFIGURATION ===");
        telemetry.addData("Selected Alliance", currentAlliance == Alliance.RED ? "🔴 RED" : "🔵 BLUE");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.startCamera();

        // Calculate Poses natively based on Alliance Choice
        if (currentAlliance == Alliance.RED) {
            startPose = redStartPose;
            targetAreaPose = redTargetAreaPose;
            loadingZoneParkPose = redLoadingZoneParkPose;
        } else {
            startPose = new Pose(redStartPose.x(), 144.0 - redStartPose.y(), -redStartPose.heading());
            targetAreaPose = new Pose(redTargetAreaPose.x(), 144.0 - redTargetAreaPose.y(), -redTargetAreaPose.heading());
            loadingZoneParkPose = new Pose(redLoadingZoneParkPose.x(), 144.0 - redLoadingZoneParkPose.y(), -redLoadingZoneParkPose.heading());
        }

        follower.setPose(startPose);

        // Build the Absolute Approach Path
        absolutePreDrivePath = line(startPose, targetAreaPose).linear(startPose, targetAreaPose);

        // 1. Command A: Drive to the absolute zone close to targets
        Command driveToZone = Command.build()
                .setStart(() -> {
                    telemetry.addLine("Executing absolute approach path...");
                    follower.follow(absolutePreDrivePath);
                })
                .setDone(() -> !follower.isBusy());

        // 2. Command B: Dynamic Vision Search & Alignment

        Command limelightAlignAndDrive = Command.build()
                .setStart(() -> {
                    alignStartMs = System.currentTimeMillis();
                    alignmentPath = null;
                })
                .setExecute(() -> {
                    if (alignmentPath == null && vision.hasTarget()
                            && vision.getLatestTarget().isValid()) {
                        LLResult result = vision.getLatestTarget();
                        if (Math.abs(result.getTx()) < 15) {
                            Pose currentPose = follower.pose();
                            Pose target = new Pose(currentPose.x() + 12.0, currentPose.y(), currentPose.heading());
                            alignmentPath = line(currentPose, target).linear(currentPose, target);
                            follower.follow(alignmentPath);
                        }
                    }
                })
                .setDone(() -> (alignmentPath != null && !follower.isBusy())
                        || System.currentTimeMillis() - alignStartMs > 3000)
                .setEnd(end -> telemetry.addLine("Alignment done or timed out."));

        // 3. Command C: Firing Chain Sequence using your Subsystem Methods
        Command shootAllBalls = sequential(
                shooter.spinUp(),
                waitMs(800),
                shooter.feedPollen(),
                waitMs(3000),
                shooter.stopIndexer(),
                shooter.stopFlywheel()
        );

        // 4. Command D: Move from dynamic post-alignment spot to the correct Loading Zone
        Command driveToPark = Command.build()
                .setStart(() -> {
                    Pose postShootPose = follower.pose();
                    parkPath = line(postShootPose, loadingZoneParkPose).linear(postShootPose, loadingZoneParkPose);
                    follower.follow(parkPath);
                })
                .setDone(() -> !follower.isBusy());

        // 5. Command E: Save the exact final position to OpModeStorage for TeleOp handoff
        Command saveFinalPose = instant(() -> {
            OpModeStorage.autonomousEndPose = follower.pose();
            telemetry.addLine("Final Pose Stored Successfully!");
        });

        // 6. Master Chaining: Drive ➡️ Align ➡️ Shoot ➡️ Park ➡️ Save Position
        Scheduler.schedule(
                sequential(
                        driveToZone,
                        limelightAlignAndDrive,
                        shootAllBalls,
                        driveToPark,
                        saveFinalPose
                )
        );
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("Alliance Status", currentAlliance == Alliance.RED ? "🔴 RED" : "🔵 BLUE");
        telemetry.addData("Robot Pose", follower.pose().toString());
        telemetry.update();
    }
}
