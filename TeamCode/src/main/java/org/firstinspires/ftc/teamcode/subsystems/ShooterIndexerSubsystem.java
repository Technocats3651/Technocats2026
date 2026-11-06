package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class ShooterIndexerSubsystem {
    private final DcMotorEx flywheel;
    private final CRServo indexer;

    // Constants for your BioBuzz setup
    private static final double SHOOTING_POWER = 0.85;
    private static final double INDEXER_FEED_POWER = 1.0;
    private static final double INDEXER_STOP_POWER = 0.0;

    public ShooterIndexerSubsystem(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        indexer = hardwareMap.get(CRServo.class, "indexer"); // Your CR Servo indexer
    }

    // Command to rev up the shooter flywheel
    public Command spinUp() {
        return Command.build()
                .setStart(() -> flywheel.setPower(SHOOTING_POWER))
                .setDone(() -> true) // Finishes instantly, letting the motor rev in the background
                .requiring(flywheel);
    }

    // Command to stop the flywheel
    public Command stopFlywheel() {
        return Command.build()
                .setStart(() -> flywheel.setPower(0))
                .setDone(() -> true)
                .requiring(flywheel);
    }

    // Command to feed pollen via the CR Servo Indexer
    public Command feedPollen() {
        return Command.build()
                .setStart(() -> indexer.setPower(INDEXER_FEED_POWER))
                .setDone(() -> true)
                .requiring(indexer);
    }

    // Command to stop the CR Servo Indexer
    public Command stopIndexer() {
        return Command.build()
                .setStart(() -> indexer.setPower(INDEXER_STOP_POWER))
                .setDone(() -> true)
                .requiring(indexer);
    }

    // A unified macro command to execute a single cycle shot
    public Command fireShot() {
        return sequential(
                spinUp(),
                waitMs(800),       // Allow time for the flywheel to reach target velocity
                feedPollen(),
                waitMs(500),       // Time duration required to push item completely through
                stopIndexer(),
                stopFlywheel()
        );
    }
}
