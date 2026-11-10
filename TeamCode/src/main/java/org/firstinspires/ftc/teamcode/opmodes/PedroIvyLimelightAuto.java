package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.api.Paths.*;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous(name = "Ivy Pedro Limelight Auto Example", group = "Autonomous")
public class PedroIvyLimelightAuto extends OpMode {

    private Follower follower;
    private VisionSubsystem vision;

    private IntakeSubsystem intake;
    private Path alignmentPath;

    // Starting coordinate posture
    private final Pose startPose = new Pose(24, 24, Math.toRadians(0));

    @Override
    public void init() {
        // Initialize Scheduler
        Scheduler.reset();

        // 1. Initialize Pedro Pathing follower via your tuned Constants
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        // 2. Initialize Limelight
        intake = new IntakeSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        vision.switchPipeline(0); // e.g., Pipeline 0 for Color Tracking or AprilTags

        telemetry.addData("Status", "Initialized Elements");
    }

    @Override
    public void start() {
        vision.startCamera(); // Turn camera stream on to save power during init

        // 1. Swap 'new CommandBuilder()' to the official 'Command.build()' factory method
        Command limelightAlignAndDrive = Command.build()
                .setStart(() -> {
                    telemetry.addLine("Searching for vision targets...");
                })
                .setExecute(() -> {
                    // REMOVED follower.update() from here since it runs continuously in loop()

                    if (vision.hasTarget()) {
                        com.qualcomm.hardware.limelightvision.LLResult result = vision.getLatestTarget();
                        double tx = result.getTx(); // Horizontal offset
                        double ty = result.getTy(); // Vertical / Distance offset

                        telemetry.addData("Target X Offset", tx);
                        telemetry.addData("Target Y Offset", ty);

                        // If a target is acquired and requires a re-route:
                        if (alignmentPath == null && Math.abs(tx) < 15) {
                            // Capture current pose cleanly
                            Pose currentPose = follower.pose();

                            // Calculate target pose relative to current location
                            Pose targetPose = new Pose(currentPose.x() + 12.0, currentPose.y(), currentPose.heading());

                            // Build the straight alignment path natively using v3 syntax
                            alignmentPath = line(currentPose, targetPose)
                                    .linear(currentPose, targetPose);

                            // Send the new path to the follower
                            follower.follow(alignmentPath);
                        }
                    }
                })
                .setDone(() -> {
                    // Query the subsystem directly so it has access to a fresh, up-to-date result
                    return vision.getLatestTarget().isValid();
                })
                .setEnd((endCondition) -> {
                    telemetry.addLine("Vision Alignment Command Complete.");
                }); // 👈 Command.build() chains are already fully formed commands!

        // Schedule the command onto Ivy's Scheduler engine
        Scheduler.schedule(limelightAlignAndDrive);
    }


    @Override
    public void loop() {
        // 1. Update the Pedro follower movement loops
        follower.update();

        // 2. Continuous cooperative execution of all scheduled Ivy commands
        Scheduler.execute();

        // 3. Telemetry updates (using the updated .pose() method name for v3)
        telemetry.addData("Robot Pose", follower.pose().toString());
        telemetry.update();
    }
}

