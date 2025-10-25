package org.firstinspires.ftc.teamcode.pedroPathing;

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
    /** Setup:
     * Pinpoint Odometry
     * Mecanum Drive
     **/

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants blueDriveConstants = new MecanumConstants()
            .maxPower(1)
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            // Directions for BLUE ALLIANCE
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static MecanumConstants redDriveConstants = new MecanumConstants()
            .maxPower(1)
            // These names are inverse for RED ALLIANCE
            .leftRearMotorName("rightFront")
            .leftFrontMotorName("rightBack")
            .rightFrontMotorName("leftRear")
            .rightRearMotorName("leftFront")
            // Directions for BLUE ALLIANCE
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            // TODO: Adjust these offsets
            .forwardPodY(-5)
            .strafePodX(0.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // TODO: Reverse needed pods
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return createFollower(hardwareMap, true);
    }

    public static Follower createFollower(HardwareMap hardwareMap, boolean isBlueSide) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(isBlueSide? blueDriveConstants : redDriveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
