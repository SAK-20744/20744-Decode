package org.firstinspires.ftc.teamcode.subsystems.pedroPathing;

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
            .mass(23);
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .rightFrontMotorName(ApolloConstants.dt.fr)
            .rightRearMotorName(ApolloConstants.dt.br)
            .leftRearMotorName(ApolloConstants.dt.bl)
            .leftFrontMotorName(ApolloConstants.dt.fl)
            .leftFrontMotorDirection(ApolloConstants.flDir)
            .leftRearMotorDirection(ApolloConstants.blDir)
            .rightFrontMotorDirection(ApolloConstants.frDir)
            .rightRearMotorDirection(ApolloConstants.brDir);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5)
            .strafePodX(7.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
