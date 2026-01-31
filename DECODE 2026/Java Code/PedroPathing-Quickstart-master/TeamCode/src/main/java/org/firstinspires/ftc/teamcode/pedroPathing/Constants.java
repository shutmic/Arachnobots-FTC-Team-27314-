package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    static double forwardTunerMultiplier = 0.014644892268754245;
    static double lateralTunerMultiplier = 0.03834525377417162;
    static double turnTunerMultiplier = 0.014864056361111087;
    static double xVelocity = 84.04234627200739;
    static double yVelocity = 72.21761597513971;
    static double forwardDeceleration = -139.53206319225737;
    static double lateralDeceleration = -877.6451014469201;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(forwardDeceleration)
            .lateralZeroPowerAcceleration(lateralDeceleration)
            .mass(9);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(xVelocity) // Forward Velocity Tuner
            .yVelocity(yVelocity); // Lateral Velocity Tuner

    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()

            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.REVERSE)
            .rightRearEncoderDirection(Encoder.REVERSE)
            .robotWidth(17)
            .robotLength(17)

            // Multipliers for Tuning
            .forwardTicksToInches(forwardTunerMultiplier)
            .strafeTicksToInches(lateralTunerMultiplier)
            .turnTicksToInches(turnTunerMultiplier)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)

                .build();
    }
}
