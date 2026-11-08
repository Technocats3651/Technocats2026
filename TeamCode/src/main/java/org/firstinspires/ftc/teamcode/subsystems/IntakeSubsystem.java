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

    private boolean isActive;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "FL_Intake_Servo");
        rightServo = hardwareMap.get(CRServo.class, "FR_Intake_Servo");
        frontRoller = hardwareMap.get(DcMotorEx.class, "Intake");

        // Reverse one side if the servos face opposite directions
        rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        isActive = false;
    }

    /**
     * Direct hardware control method.
     * Use this for direct trigger mapping in TeleOp loops.
     */
    public void setMotorPower(double power) {
        leftServo.setPower(power);
        rightServo.setPower(power);
        frontRoller.setPower(power);
        isActive = Math.abs(power) > 0.05; // Mark active if power is non-zero
    }

    /**
     * Continuous command to run the entire intake mechanism at a given speed.
     */
    public Command runIntake(double power) {
        return com.pedropathing.ivy.Command.build()
                .setExecute(() -> setMotorPower(power))
                .setEnd(endCondition -> setMotorPower(0.0));
    }

    public boolean isActive(){
        return isActive;
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
