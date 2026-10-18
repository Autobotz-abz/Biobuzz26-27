package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.70268)
            .forwardZeroPowerAcceleration(30.1)
            .lateralZeroPowerAcceleration(49.926)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1, 0, 0.003, 0.02
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1, 0, 0.0006, 0.03
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.12, 0.01, 0.004, 0.9, 0.015
            ))
            .centripetalScaling(0.0004);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.75)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(64.41)   // tuned for 312 rpm + 104mm wheels
            .yVelocity(51.92);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3.34646)   // inches
            .strafePodX(-6.1)    // inches
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(
            0.9,
            100,
            1,
            1
    );
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}