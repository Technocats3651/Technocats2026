package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utility.MotorEx;

//@Config
public class Flywheel {
    private MotorEx shooterMotorA;

    public static double kS = 0.08, kV = 0.000375, kP = 0.003; //TODO: TUNE VALUES

    private double target = 0;
    private boolean activated = false;

    VoltageSensor voltageSensor;

    public Flywheel(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;

        shooterMotorA = new MotorEx(hardwareMap, "Flywheel");

        shooterMotorA.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    /**
     *
     * @param ticksPerSecond motor speed in ticks per second
     * @return flywheel speed in radians per second
     */
    public static double motorTicksToFlywheelRadians(double ticksPerSecond) {
        //28 -- ticks per rotation
        // 1.4 -- ratio of flywheel velocity to motor velocity
        return (ticksPerSecond / 28) * 2 * Math.PI * 1.4;
    }

    /**
     *
     * @param radiansPerSecond flywheel speed in radians per second
     * @return motor speed in ticks per second
     */
    public static double flywheelRadiansToMotorTicks(double radiansPerSecond) {
        //28 -- ticks per rotation
        // 1.4 -- ratio of flywheel velocity to motor velocity
        return (radiansPerSecond / (2 * Math.PI * 1.4)) * 28;
    }


    public double getCurrentAngularVel() {
        return shooterMotorA.getVelocity();
    }

    public void setTargetAngularVelocity(double target) {
        this.target = target;
    }


    public double getTargetAngularVelocity() {
        return target;
    }

    public void setPower(double power) {
        shooterMotorA.setPower(power);

    }

    public void deactivate() {
        activated = false;
        setPower(0);
    }

    public void activate() {
        activated = true;
    }

    public boolean isReady() {
        return Math.abs(getTargetAngularVelocity() - getCurrentAngularVel()) <= 40;
    }

    public boolean getActivated() {
        return activated;
    }

    public double getCurrent() {
        return shooterMotorA.getCurrent();
    }

    public void toggle() {
        activated = !activated;
        if (!activated) {
            setPower(0);
        }
    }

    public void update() {
        if (activated) {
            double power =  (kV * getTargetAngularVelocity()) + (kP * (getTargetAngularVelocity() - getCurrentAngularVel())) + kS;
            power *= 12 / voltageSensor.getVoltage();
            setPower(power);
        }
    }
}

