package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
//import com.pedropathing.pathogen.*;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.pathgen.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterIndexerSubsystem;

// Important: Static imports for Ivy control flow structures and Pedro commands
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;              // Correct package for Path in v3
import static com.pedropathing.api.Paths.*;     // Gives you the line() and curve() methods
import com.pedropathing.api.PoseFactory;
@Autonomous(name = "BioBuzz Ivy Autonomous Flow")
public class IvyAutonomous extends LinearOpMode {
    private final com.pedropathing.api.PoseFactory p = com.pedropathing.api.PoseFactory.degrees();
    private Follower follower;
    private ShooterIndexerSubsystem shooter;
    private Limelight3A limelight;
    private IntakeSubsystem intake;

    @Override
    public void runOpMode() {
        // 1. Always reset the static Ivy scheduler before starting an OpMode
        Scheduler.reset();

        // 2. Initialize Subsystems & Hardware Map Configurations
        follower = Constants.create(hardwareMap);
        shooter = new ShooterIndexerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Set pipeline (e.g., 0 for Yellow Pollen tracking)

        // 3. Define Poses and Paths
        Pose startPose = new Pose(12.0, 12.0, 0.0);
        follower.setPose(startPose);

        // Uses the PoseFactory (p) to turn 60.0, 24.0, 0.0 into a proper Pose object
        Path scorePreload = line(startPose, p.of(60.0, 24.0, 0.0))
                .linear(startPose, p.of(60.0, 24.0, 0.0));

        // 4. Construct the Command Composition Sequence
        Command autonomousRoutine = sequential(
                // Simultaneously drive to position while spinning up the shooter's flywheel
                parallel(
                        follow(follower, scorePreload), // Pedro Ivy Command wrapper
                        shooter.spinUp()
                ),

                // Vision alignment step using your Limelight
                Command.build()
                        .setExecute(() -> {
                            LLResult result = limelight.getLatestResult();
                            if (result != null && result.isValid()) {
                                double tx = result.getTx(); // Target X offset
                                // Optional: Inject localized adjustment powers directly to your drivetrain motors here
                            }
                        })
                        .setDone(() -> {
                            LLResult result = limelight.getLatestResult();
                            // Finish command when vision alignment is accurate within an acceptable error window
                            return result != null && Math.abs(result.getTx()) < 1.5;
                        }),

                // Actuate the CR Servo indexer to complete the launch cycle
                shooter.fireShot()
        );

        // Wait for user payload start
        limelight.start(); // Conserve processing overhead until the match starts
        waitForStart();

        // 5. Schedule the primary routine command
        Scheduler.schedule(autonomousRoutine);

        // 6. Main execution engine loop
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();      // Update underlying localized odometry matrices
            Scheduler.execute();   // Process the current active node step in the command queue
        }
    }
}

