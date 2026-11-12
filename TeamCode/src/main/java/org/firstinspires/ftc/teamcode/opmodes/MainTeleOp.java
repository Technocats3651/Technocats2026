package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStorage;
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

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;
    private boolean bLastInit = false;
    private boolean xLastInit = false;

    // Hive centers in field inches (measure these for each alliance)
    private static final double RED_HIVE_X = 570.0, RED_HIVE_Y = 70.0;
    private static final double BLUE_HIVE_X = 84.0, BLUE_HIVE_Y = 70.0;

    // Calibration table: distance (inches) -> flywheel power
    private static final double[] DIST_IN = {36, 60, 84, 108};
    private static final double[] POWER   = {0.65, 0.75, 0.85, 0.95};

    private double headingOffset = 0.0;   // radians
    private boolean startLast = false;

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
        // Use the pose saved by the autonomous, or the origin if none was saved
        Pose startPose = OpModeStorage.autonomousEndPose != null
                ? OpModeStorage.autonomousEndPose
                : new Pose(0.0, 0.0, 0.0);
        follower.setPose(startPose);
        // Hardware subsystem mappings
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterIndexerSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        alliance = OpModeStorage.isRedAlliance ? Alliance.RED : Alliance.BLUE;
    }

    @Override
    public void init_loop() {
        if (gamepad1.b && !bLastInit) alliance = Alliance.RED;
        if (gamepad1.x && !xLastInit) alliance = Alliance.BLUE;
        bLastInit = gamepad1.b;
        xLastInit = gamepad1.x;

        telemetry.addData("Alliance", alliance == Alliance.RED ? "RED (B)" : "BLUE (X)");
        telemetry.addData("Press B for red, X for blue", "");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.startCamera(); // Fire up Limelight stream processing
    }

    @Override
    public void loop() {

        // Driver presses START (Options on some controllers) while the robot faces away from them
        if (gamepad1.start && !startLast) {
            headingOffset = follower.pose().heading();
        }
        startLast = gamepad1.start;
        // ==========================================
        // GAMEPAD 1: DRIVETRAIN EXTENSION
        // ==========================================
        // Control translational and rotational mechanics using joysticks
        // follower.setTeleOpMovementVectors(forward, strafe, turn, fieldCentric)
        // Generate field-centric drive power vectors dynamically using your heading
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading() - headingOffset
        );

        // Pass the vectors to the manual driver
        follower.manual(powers);


        // Always call your path/drivetrain updater first
        follower.update();

        // ==========================================
        // GAMEPAD 2: AUXILIARY INTAKE SYSTEMS
        // ==========================================

        if (gamepad2.right_trigger > 0.1) {
            intake.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > 0.1) {
            intake.setPower(-gamepad2.left_trigger);
        } else {
            intake.setPower(0.0);
        }

        // ==========================================
        // GAMEPAD 2: SHOOTER & FLYWHEEL CONTROL
        // ==========================================
        // Toggle the flywheel on/off using a clean boolean check on Circle/B button
        if (gamepad2.circle && !cross2Last) {
            isShooterActive = !isShooterActive;
            if (!isShooterActive) {
                shooter.setFlywheelPower(0.0);
            }
        }
        cross2Last = gamepad2.circle;

        if (isShooterActive) {
            double dist = vision.distanceToTagsInches(hiveTags());
            if (Double.isNaN(dist)) {
                double hiveX = (alliance == Alliance.RED) ? RED_HIVE_X : BLUE_HIVE_X;
                double hiveY = (alliance == Alliance.RED) ? RED_HIVE_Y : BLUE_HIVE_Y;
                dist = Math.hypot(hiveX - follower.pose().x(), hiveY - follower.pose().y());
            }
            shooter.setFlywheelPower(powerForDistance(dist));
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
        telemetry.addData("Shot distance (in)", vision.distanceToTagsInches(hiveTags()));
        telemetry.update();
    }

    private static double powerForDistance(double d) {
        if (d <= DIST_IN[0]) return POWER[0];
        if (d >= DIST_IN[DIST_IN.length - 1]) return POWER[POWER.length - 1];
        for (int i = 0; i < DIST_IN.length - 1; i++) {
            if (d <= DIST_IN[i + 1]) {
                double t = (d - DIST_IN[i]) / (DIST_IN[i + 1] - DIST_IN[i]);
                return POWER[i] + t * (POWER[i + 1] - POWER[i]);
            }
        }
        return POWER[POWER.length - 1];
    }
    private int[] hiveTags() {
        return alliance == Alliance.RED ? VisionSubsystem.RED_HIVE_TAGS : VisionSubsystem.BLUE_HIVE_TAGS;
    }
}

