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

    private final com.pedropathing.api.PoseFactory p = com.pedropathing.api.PoseFactory.degrees();
    private final com.pedropathing.math.Pose startPose = p.of(0.0, 0.0, 0.0);

    // Runtime state variables to handle button edge-triggers safely
    private boolean rightBumper2Last = false;
    private boolean cross2Last = false;
    private boolean isShooterActive = false;

    private Command currentFlywheelCommand = null;
    private Command intakeControlCommand = null; // Class variable to track the intake routine

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

        // Define the continuous loop command using Ivy's 'infinite' syntax
        // This will now perfectly recognize the .setMotorPower method we added above!
        intakeControlCommand = com.pedropathing.ivy.commands.Commands.infinite(() -> {
            if (gamepad2.right_trigger > 0.1) {
                intake.setMotorPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setMotorPower(-gamepad2.left_trigger);
            } else {
                intake.setMotorPower(0.0);
            }
        }); // Removed .requiring(intake) since we are managing it directly via start()
    }

    @Override
    public void start() {
        vision.startCamera(); // Fire up Limelight stream processing

        // Manually schedule the trigger processing loop when TeleOp begins
        com.pedropathing.ivy.Scheduler.schedule(intakeControlCommand);
    }

    @Override
    public void loop() {
        // ==========================================
        // GAMEPAD 1: DRIVETRAIN EXTENSION
        // ==========================================
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading()
        );

        follower.manual(powers);
        follower.update();

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

        // Shoot macro (Right Bumper)
        if (gamepad2.right_bumper && !rightBumper2Last) {
            Command shoot = shooter.shootMacro();
            Scheduler.schedule(shoot);
        }
        rightBumper2Last = gamepad2.right_bumper;

        // ==========================================
        // PERSISTENT PROCESSING ENGINE
        // ==========================================
        // Executes a single cycle of all running parallel commands (including Intake!)
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

