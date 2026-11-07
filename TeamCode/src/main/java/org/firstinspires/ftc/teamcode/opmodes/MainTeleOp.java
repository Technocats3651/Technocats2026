package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterIndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;


@TeleOp(name = "Mecanum Ivy Dual TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private Follower follower;
    private IntakeSubsystem intake;
    private ShooterIndexerSubsystem shooter;
    private VisionSubsystem vision;

    // Add these two lines at the top of your class:
    private final com.pedropathing.api.PoseFactory p = com.pedropathing.api.PoseFactory.degrees();
    private final com.pedropathing.math.Pose startPose = p.of(0.0, 0.0, 0.0); // Defaults to center origin

    // Runtime state variables to handle button edge-triggers safely
    private boolean rightBumper2Last = false;
    private boolean cross2Last = false;
    private boolean isShooterActive = false;

    private Command currentFlywheelCommand = null;

    @Override
    public void init() {
        // Reset the static Ivy control engine
        Scheduler.reset();

        // Factory initialization for Pedro v3 Follower mechanics
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        // Hardware subsystem mappings
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterIndexerSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
    }

    @Override
    public void start() {
        vision.startCamera(); // Fire up Limelight stream processing
    }

    @Override
    public void loop() {
        // ==========================================
        // GAMEPAD 1: DRIVETRAIN EXTENSION
        // ==========================================
        // Control translational and rotational mechanics using joysticks
        // follower.setTeleOpMovementVectors(forward, strafe, turn, fieldCentric)
        // Generate field-centric drive power vectors dynamically using your heading
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );

// Pass the vectors to the manual driver
        follower.manual(powers);


        // Always call your path/drivetrain updater first
        follower.update();

        // ==========================================
        // GAMEPAD 2: AUXILIARY INTAKE SYSTEMS
        // ==========================================
        // Right trigger actively intakes, Left trigger outtakes (reverses)
        if (gamepad2.right_trigger > 0.1) {
            intake.runIntake(gamepad2.right_trigger); // Scales speed based on analog pull
        } else if (gamepad2.left_trigger > 0.1) {
            intake.runIntake(-gamepad2.left_trigger); // Outtake configuration
        } else {
            intake.runIntake(0.0); // Stop both side servos and front roller if nothing is pressed
        }

        // ==========================================
        // GAMEPAD 2: SHOOTER & FLYWHEEL CONTROL
        // ==========================================
        // Toggle the flywheel on/off using a clean boolean check on Circle/B button
        if (gamepad2.circle && !cross2Last) {
            isShooterActive = !isShooterActive;

            if (isShooterActive) {
                currentFlywheelCommand = shooter.runFlywheel(0.9); // Spin up to 90% power
                Scheduler.schedule(currentFlywheelCommand);
            } else if (currentFlywheelCommand != null) {
                Scheduler.cancel(currentFlywheelCommand);
                currentFlywheelCommand = null;
            }
        }
        cross2Last = gamepad2.circle;

        // Shoot macro (Right Bumper) - pushes the indexer servo out and pulls it back
        // Checks that the shoot macro isn't already active to prevent multi-scheduling
        if (gamepad2.right_bumper && !rightBumper2Last) {
            Command shoot = shooter.shootMacro();
            Scheduler.schedule(shoot);
        }
        rightBumper2Last = gamepad2.right_bumper;

        // ==========================================
        // PERSISTENT PROCESSING ENGINE
        // ==========================================
        // Executes a single cycle of all running parallel commands
        Scheduler.execute();

        // Telemetry diagnostics
        telemetry.addData("X Position", follower.pose().x());
        telemetry.addData("Y Position", follower.pose().y());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Flywheel Status", isShooterActive ? "ENGAGED" : "OFF");
        telemetry.addData("Limelight Valid Target", vision.hasTarget());
        telemetry.addData("Limelight intake", intake.isActive()? "ON" : "OFF");
        telemetry.update();
    }
}

