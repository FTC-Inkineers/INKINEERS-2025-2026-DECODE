package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class DriveSubsystem {
    // Tunable proportional gains for AprilTag alignment
    private static final double kDrive = 0.02;     // Forward/backward
    private static final double kStrafe = 0.02;    // Left/right
    private static final double kTurn = 0.02;      // Rotation
    
    // PEDRO PATHING
    public final Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private final TelemetryManager telemetryM;

    // TELEOP
    private Gamepad gamepad;
    ComputerVision CV;

    InputRamper forwardRamper, strafeRamper, turnRamper;

    public DriveSubsystem(HardwareMap hardwareMap, boolean isBlueSide) {

        follower = Constants.createFollower(hardwareMap, isBlueSide);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        CV = new ComputerVision(hardwareMap);
    }

    public void initTeleOp(Gamepad gamepad1) {
        this.gamepad = gamepad1;

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
    
    public void runTeleOp(boolean shooterIsActive) {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        double leftYInput = forwardRamper.rampInput(gamepad.left_stick_y);
        double leftXInput = strafeRamper.rampInput(gamepad.left_stick_x);
        // Needs to be less sensitive for turning.
        double rightXInput = turnRamper.rampInput(gamepad.right_stick_x) * (shooterIsActive ? 0.4 : 0.8);

        // Last parameter --- True: Robot Centric | False: Field Centric
        follower.setTeleOpDrive(
                -leftYInput, // Forward
                -leftXInput, // Strafe
                -rightXInput, // Turn
                automatedDrive // Field Centric
        );

        // Reset Pose
        if (gamepad.backWasPressed()) {
            follower.setPose(new Pose(0, 0, 0));
        }

        // Hold Y to align
        if (gamepad.y) {
            automatedDrive = alignToAprilTag(CV.getLLResult());
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }

    public void enableAllTelemetry(OpMode opMode, boolean enableAll) {
        opMode.telemetry.addLine("\\ DRIVE //");
        if (enableAll) {
            opMode.telemetry.addData("position", follower.getPose());
            opMode.telemetry.addData("velocity", follower.getVelocity());
        }
        opMode.telemetry.addData("automatedDrive", automatedDrive);

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

    public boolean alignToPose(double x, double y, double z, double targetDistance) {
        double xError = x - targetDistance;     // Positive = too far

        // Calculate movement commands
        double drive = xError * kDrive;
        double strafe = y * kStrafe;
        double turn = z * kTurn;

        // Clamp speeds at 0.4 max/min
        drive = Math.max(-0.4, Math.min(0.4, drive));
        strafe = Math.max(-0.4, Math.min(0.4, strafe));
        turn = Math.max(-0.4, Math.min(0.4, turn));

        // Apply movement through Pedro follower
        follower.setTeleOpDrive(-drive, -strafe, -turn, true);

        telemetryM.debug("AlignToAprilTag", "xErr: %.2f, yErr: %.2f, yawErr: %.2f", xError, y, z);
        telemetryM.debug("DriveCmds", "f: %.2f, s: %.2f, t: %.2f", drive, strafe, turn);

        // Determine if alignment is done
        boolean aligned = Math.abs(xError) < 1.0 && Math.abs(y) < 1.0 && Math.abs(z) < 2.0;

        if (aligned) {
            follower.setTeleOpDrive(0, 0, 0, true);
        }

        return aligned;
    }

    /**
     * Align to a given AprilTagPose object.
     * This can be called from a VisionSubsystem that provides tag data.
     */
    public boolean alignToAprilTag(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        LLResultTypes.FiducialResult tagPose = null; // Initialize to null

        if (fiducials.isEmpty()) {
            follower.setTeleOpDrive(0, 0, 0, true);
            telemetryM.debug("AlignToAprilTag", "No tags detected");
            return false; // No tags to align to
        }

        // Find the largest AprilTag in the list
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (tagPose == null || fiducial.getTargetArea() > tagPose.getTargetArea()) {
                tagPose = fiducial;
            }
        }

        // It's good practice to ensure aprilTag is not null before using it,
        // though the initial check for an empty list should prevent this.
        if (tagPose == null) {
            return false;
        }

        // Error terms. Pitch, roll, and yaw, respectively.
        Pose3D error3d = tagPose.getCameraPoseTargetSpace();
        return alignToPose(error3d.getPosition().x, error3d.getPosition().y, error3d.getPosition().z, 10.0); // Default target distance = 10 inches
    }
}
