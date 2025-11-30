package org.firstinspires.ftc.teamcode.subsystems.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.ApolloConstants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(23)
            .forwardZeroPowerAcceleration(-31.873)
            .lateralZeroPowerAcceleration(-52.664)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25,0,0.02,0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0,0.03,0));
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .rightFrontMotorName(ApolloConstants.dt.fr)
            .rightRearMotorName(ApolloConstants.dt.br)
            .leftRearMotorName(ApolloConstants.dt.bl)
            .leftFrontMotorName(ApolloConstants.dt.fl)
            .leftFrontMotorDirection(ApolloConstants.flDir)
            .leftRearMotorDirection(ApolloConstants.blDir)
            .rightFrontMotorDirection(ApolloConstants.frDir)
            .rightRearMotorDirection(ApolloConstants.brDir)
            .xVelocity(74.31)
            .yVelocity(57.35);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5)
            .strafePodX(7.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
