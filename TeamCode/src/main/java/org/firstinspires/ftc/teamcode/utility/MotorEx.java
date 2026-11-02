package org.firstinspires.ftc.teamcode.utility;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Constants;

public class MotorEx {
    private final DcMotorEx motor;
    private double cachingTolerance = Constants.MOTOR_CACHING_TOLERANCE;
    private double lastPower = Double.NaN;

    public MotorEx(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotorEx.class, name);
    }

    public MotorEx setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    public double getCachingTolerance() {
        return cachingTolerance;
    }

    public void setPower(double power) {
        if (Double.isNaN(lastPower) || Math.abs(power - lastPower) > cachingTolerance
                || (power == 0 && lastPower != 0)) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    public void setSlewPower(double power, double slewRate) {
        double currentPower = Double.isNaN(lastPower) ? 0 : lastPower;

        if (Double.isNaN(lastPower) || Math.abs(power - currentPower) > cachingTolerance
                || (power == 0 && currentPower != 0)) {
            double delta = power - currentPower;

            double slewPower = currentPower + Math.signum(delta) * Math.min(slewRate, Math.abs(delta));

            motor.setPower(slewPower);
            lastPower = slewPower;
        }
    }

    public double getPower() {
        return motor.getPower();
    }

    public void setVelocity(double velocity) {
        motor.setVelocity(velocity);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
}