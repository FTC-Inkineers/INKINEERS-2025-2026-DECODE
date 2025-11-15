package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.RobotConstants.drive;
import static org.firstinspires.ftc.teamcode.RobotConstants.forwardZeroPowerAcceleration;
import static org.firstinspires.ftc.teamcode.RobotConstants.heading;
import static org.firstinspires.ftc.teamcode.RobotConstants.lateralZeroPowerAcceleration;
import static org.firstinspires.ftc.teamcode.RobotConstants.translational;
import static org.firstinspires.ftc.teamcode.RobotConstants.xVelocity;
import static org.firstinspires.ftc.teamcode.RobotConstants.yVelocity;

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

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.80)
            .forwardZeroPowerAcceleration(forwardZeroPowerAcceleration)
            .lateralZeroPowerAcceleration(lateralZeroPowerAcceleration)
            .translationalPIDFCoefficients(translational)
            .drivePIDFCoefficients(drive)
            .headingPIDFCoefficients(heading)
            .useSecondaryDrivePIDF(false);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // --- Mecanum Drive --- //
    public static MecanumConstants blueDriveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(xVelocity)
            .yVelocity(yVelocity)

            // Configuration for BLUE Alliance
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            // Directions for BLUE ALLIANCE
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static MecanumConstants redDriveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(xVelocity)
            .yVelocity(yVelocity)

            // These names are inverse for RED ALLIANCE
            .leftRearMotorName("rightFront")
            .leftFrontMotorName("rightRear")
            .rightFrontMotorName("leftRear")
            .rightRearMotorName("leftFront")
            // Directions for RED ALLIANCE
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

//    // Testing
//    public static MecanumConstants blueTestingDriveConstants = new MecanumConstants()
//            .maxPower(1)
//            .leftRearMotorName("leftRear")
//            .leftFrontMotorName("leftFront")
//            .rightFrontMotorName("rightFront")
//            .rightRearMotorName("rightRear")
//            // Directions for BLUE ALLIANCE
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
//
//    public static MecanumConstants redTestingDriveConstants = new MecanumConstants()
//            .maxPower(1)
//            // These names are inverse for RED ALLIANCE
//            .leftRearMotorName("rightFront")
//            .leftFrontMotorName("rightRear")
//            .rightFrontMotorName("leftRear")
//            .rightRearMotorName("leftFront")
//            // Directions for RED ALLIANCE
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    // --- Pinpoint Localizer --- //
    public static PinpointConstants blueLocalizerConstants = new PinpointConstants()
            // For BLUE ALLIANCE
            .forwardPodY(125.0)
            .distanceUnit(DistanceUnit.MM)
            .strafePodX(-11.643)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // For BLUE ALLIANCE
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PinpointConstants redLocalizerConstants = new PinpointConstants()
            // For RED ALLIANCE
            .forwardPodY(-125.0)
            .distanceUnit(DistanceUnit.MM)
            .strafePodX(11.643)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // For RED ALLIANCE
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

//    public static Follower createTestingFollower(HardwareMap hardwareMap, boolean isBlueSide) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(isBlueSide ? blueTestingDriveConstants : redTestingDriveConstants)
//                .pinpointLocalizer(isBlueSide ? blueLocalizerConstants : redLocalizerConstants)
//                .build();
//    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return createFollower(hardwareMap, true);
    }

    public static Follower createFollower(HardwareMap hardwareMap, boolean isBlueSide) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(isBlueSide ? blueDriveConstants : redDriveConstants)
                .pinpointLocalizer(isBlueSide ? blueLocalizerConstants : redLocalizerConstants)
                .build();
    }
}
