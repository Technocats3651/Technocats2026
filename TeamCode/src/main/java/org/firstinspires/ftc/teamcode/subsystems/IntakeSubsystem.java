package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotorEx frontRoller;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        frontRoller = hardwareMap.get(DcMotorEx.class, "frontRoller");

        // Reverse one side if the servos face opposite directions
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Stateless command to run the entire intake mechanism at a given speed.
     */
    public Command runIntake(double power) {
        return Command.build()
                .setStart(() -> {
                    leftServo.setPower(power);
                    rightServo.setPower(power);
                    frontRoller.setPower(power);
                })
                .setEnd(endCondition -> {
                    leftServo.setPower(0);
                    rightServo.setPower(0);
                    frontRoller.setPower(0);
                }); // Do not call .build() here in Ivy v3!
    }

    /**
     * Sourced helper for an intake command that runs infinitely until interrupted.
     */
    public Command intakeActive() {
        return runIntake(1.0);
    }

    /**
     * Sourced helper for an outtake command.
     */
    public Command outtakeActive() {
        return runIntake(-0.8);
    }
}

