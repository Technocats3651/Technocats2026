package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL_Drive_Motor");
        c.frontRightName.set("FR_Drive_Motor");
        c.backLeftName.set("BL_Drive_Motor");
        c.backRightName.set("BR_Drive_Motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.2618122100830078);
        c.yPodOffset.set(-1.4786534046563575);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.19595844413706034);
                Controller secondaryTranslationalForward = Controller.proportional(0.07240140900258171);
                Controller primaryTranslationalLateral = Controller.proportional(0.363467676254768);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13429159434095814);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017380357954680722));
                c.brake.set(Controller.proportionalFeedforward(0.014773304261478614));

                c.headingFeedback.set(Controller.proportional(3.477224508430262));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.030899057172596497, 0.008863835709054545));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06387050766870242, 0.05916123083553147));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013652021490175635, 0.0020059423568949));

                c.maxAchievableForwardVelocity.set(59.001909152041414);
                c.maxAchievableStrafeVelocity.set(49.485378114345345);
                c.naturalForwardDeceleration.set(30.320937872577993);
                c.naturalStrafeDeceleration.set(74.44354864051444);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}