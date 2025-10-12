package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.openftc.apriltag.AprilTagPose;

import java.util.function.Supplier;

public class DriveSubsystem {
    // Tunable proportional gains for AprilTag alignment
    private static final double kDrive = 0.02;     // Forward/backward
    private static final double kStrafe = 0.02;    // Left/right
    private static final double kTurn = 0.02;      // Rotation
    
    // PEDRO PATHING
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private final TelemetryManager telemetryM;


    // TELEOP
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    InputRamper forwardRamper, strafeRamper, turnRamper;

    public DriveSubsystem(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    public void initTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        forwardRamper = new InputRamper();
        strafeRamper = new InputRamper();
        turnRamper = new InputRamper();
    }

    public void initAuto() {

    }
    
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    
    public void runTeleOp() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        double leftYInput = forwardRamper.rampInput(gamepad1.left_stick_y);
        double leftXInput = strafeRamper.rampInput(gamepad1.left_stick_x);
        double rightXInput = turnRamper.rampInput(gamepad1.right_stick_x);

        // Last parameter --- True: Robot Centric | False: Field Centric
        follower.setTeleOpDrive(
                -leftYInput, // Forward
                -leftXInput, // Strafe
                -rightXInput, // Turn
                automatedDrive // Field Centric
        );

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.addLine("\\ Joystick Data //");
        telemetryM.debug("Forward", leftYInput);
        telemetryM.debug("Strafe", leftXInput);
        telemetryM.debug("Turn", rightXInput);

    }

    /** @noinspection FieldCanBeLocal*/
    public static class InputRamper {
        private final double MIN_MULTIPLIER = 0.4;
        private final double  MAX_MULTIPLIER = 1.00; // Runs at 100% at full ramp
        private final double incrementMultiplier = 0.22;
        private final double timeIncrementInMs = 200;

        private final ElapsedTime rampTimer;
        private double multiplier = MIN_MULTIPLIER;

        public InputRamper() {
            rampTimer = new ElapsedTime();
        }

        public double rampInput(float input) {
            if (input != 0) {
                // Increment every 200 milliseconds
                if (rampTimer.milliseconds() > timeIncrementInMs) {
                    multiplier += incrementMultiplier;
                    rampTimer.reset();
                }
            } else if (multiplier > MIN_MULTIPLIER && rampTimer.milliseconds() > timeIncrementInMs) {
                multiplier -= incrementMultiplier;
                rampTimer.reset();
            } else {
                multiplier = MIN_MULTIPLIER;
            }

            // Do not exceed limits
            multiplier = Math.max(MIN_MULTIPLIER, Math.min(multiplier, MAX_MULTIPLIER));

            // Return accelerated output
            return input * multiplier;
        }
    }

    public boolean alignToAprilTag(AprilTagPose tagPose, double targetDistance) {
        // Error terms. Pitch, roll, and yaw, respectively.
        double xError = tagPose.x - targetDistance;     // Positive = too far
        double yError = tagPose.y;                      // Positive = robot is left of tag
        double yawError = tagPose.z;                    // Positive = rotated CCW

        // Calculate movement commands
        double drive = xError * kDrive;
        double strafe = yError * kStrafe;
        double turn = yawError * kTurn;

        // Clamp speeds at 0.4 max/min
        drive = Math.max(-0.4, Math.min(0.4, drive));
        strafe = Math.max(-0.4, Math.min(0.4, strafe));
        turn = Math.max(-0.4, Math.min(0.4, turn));

        // Apply movement through Pedro follower
        follower.setTeleOpDrive(-drive, -strafe, -turn, true);

        telemetryM.debug("AlignToAprilTag", "xErr: %.2f, yErr: %.2f, yawErr: %.2f", xError, yError, yawError);
        telemetryM.debug("DriveCmds", "f: %.2f, s: %.2f, t: %.2f", drive, strafe, turn);

        // Determine if alignment is done
        boolean aligned = Math.abs(xError) < 1.0 && Math.abs(yError) < 1.0 && Math.abs(yawError) < 2.0;

        if (aligned) {
            follower.setTeleOpDrive(0, 0, 0, true);
        }

        return aligned;
    }

    /**
     * Align to a given AprilTagPose object.
     * This can be called from a VisionSubsystem that provides tag data.
     */
    public boolean alignToAprilTag(AprilTagPose tagPose) {
        if (tagPose == null) {
            follower.setTeleOpDrive(0, 0, 0, true);
            telemetryM.debug("AlignToAprilTag", "No tag detected");
            return false;
        }
        return alignToAprilTag(tagPose, 10.0); // Default target distance = 10 inches
    }
}
