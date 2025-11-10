package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveSubsystem {
    // Tunable proportional gains for AprilTag alignment
    private static final double kP_drive = 0.02, kD_drive = 0.00;       // Forward/backward
    private static final double kP_strafe = 0.02, kD_strafe = 0.00;     // Left/right
    private static final double kP_turn = 0.02, kD_turn = 0.00;         // Turn

    private final FPIDController driveController = new FPIDController.Builder(kP_drive).withD(kD_drive).build();
    private final FPIDController strafeController = new FPIDController.Builder(kP_strafe).withD(kD_strafe).build();
    private final FPIDController turnController = new FPIDController.Builder(kP_turn).withD(kD_turn).build();
    
    // PEDRO PATHING
    public final Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private final TelemetryManager telemetryM;

    // TELEOP
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private final ComputerVision CV;

    InputRamper forwardRamper, strafeRamper, turnRamper;

    public DriveSubsystem(HardwareMap hardwareMap, boolean isBlueSide) {

        follower = Constants.createFollower(hardwareMap, isBlueSide);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        CV = new ComputerVision(hardwareMap);
    }

    public void initTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
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

    private boolean lockedOn = false;
    public void runTeleOp(boolean shooterIsActive) {
        // Call this once per loop
        follower.update();
        CV.update();
        telemetryM.update();
        driveController.setGains(0, kP_drive, 0, kD_drive);
        strafeController.setGains(0, kP_strafe, 0, kD_strafe);
        turnController.setGains(0, kP_turn, 0, kD_turn);

        // Hold Y to align
        if (gamepad1.y || gamepad2.y) {
            automatedDrive = alignToAprilTag(CV.getLargestTagPose());
        }
        // Lock on to goal (toggle).
        if (gamepad1.yWasReleased() || gamepad2.yWasReleased()) {
            lockedOn = !lockedOn;
        }

        double leftYInput = forwardRamper.rampInput(gamepad1.left_stick_y);
        double leftXInput = strafeRamper.rampInput(gamepad1.left_stick_x);
        double rightXInput;

        if (lockedOn) {
            // Auto-align.
            rightXInput = getAprilTagTurnCommand();
        } else {
            // Needs to be less sensitive for turning.
            rightXInput = turnRamper.rampInput(gamepad1.right_stick_x) * (shooterIsActive ? 0.4 : 0.8);
        }

        // Last parameter --- True: Robot Centric | False: Field Centric
        follower.setTeleOpDrive(
                -leftYInput, // Forward
                -leftXInput, // Strafe
                -rightXInput, // Turn
                automatedDrive // Field Centric when lockedOn
        );

        // Reset Pose
        if (gamepad1.backWasPressed()) {
            follower.setPose(new Pose(0, 0, 0));
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }
    }

    public void runTeleOp() {
        runTeleOp(false);
    }

    public void sendAllTelemetry(Telemetry telemetry, boolean enableAll) {
        telemetry.addLine("\\ DRIVE //");
        if (enableAll) {
            telemetry.addData("position", follower.getPose());
            telemetry.addData("velocity", follower.getVelocity());
        }
        telemetry.addData("automatedDrive", automatedDrive);
        telemetry.addData("lockedOn", lockedOn);
    }

    public void sendAprilTagTelemetry(Telemetry telemetry) {
        CV.sendTelemetry(telemetry);
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

    private double getAprilTagTurnCommand() {
        Pose3D tagPose = CV.getLargestTagPose();

        if (tagPose == null) {
            turnController.reset();
            return 0.0;
        }

        // Error is yaw.
        double currentError = tagPose.getPosition().z;
        FPIDOutput turnOutput = turnController.calculate(currentError);
        double turn = turnOutput.total;

        // Clamp speed
        turn = Math.max(-0.4, Math.min(0.4, turn));

        telemetryM.debug("LockOn", "yawErr: %.2f", currentError);
        telemetryM.debug("LockOnPID", "P: %.2f, I: %.2f, D: %.2f, Total: %.2f",
                turnOutput.p, turnOutput.i, turnOutput.d, turn);

        return turn;
    }

    public boolean alignToPose(double x, double y, double z, double targetDistance) {
        double xError = x - targetDistance; // Positive = too far

        // Calculate movement commands and get full PID output for each
        FPIDOutput driveOutput = driveController.calculate(xError);
        FPIDOutput strafeOutput = strafeController.calculate(y);
        FPIDOutput turnOutput = turnController.calculate(z);

        double drive = driveOutput.total;
        double strafe = strafeOutput.total;
        double turn = turnOutput.total;

        // Clamp speeds at 0.4 max/min
        drive = Math.max(-0.4, Math.min(0.4, drive));
        strafe = Math.max(-0.4, Math.min(0.4, strafe));
        turn = Math.max(-0.4, Math.min(0.4, turn));

        follower.setTeleOpDrive(-drive, -strafe, -turn, true);

        // DETAILED TELEMETRY
        telemetryM.debug("AlignErrors", "xErr: %.2f, yErr: %.2f, yawErr: %.2f", xError, y, z);
        telemetryM.debug("DrivePID", "P: %.2f, I: %.2f, D: %.2f, Total: %.2f",
                driveOutput.p, driveOutput.i, driveOutput.d, drive);
        telemetryM.debug("StrafePID", "P: %.2f, I: %.2f, D: %.2f, Total: %.2f",
                strafeOutput.p, strafeOutput.i, strafeOutput.d, strafe);
        telemetryM.debug("TurnPID", "P: %.2f, I: %.2f, D: %.2f, Total: %.2f",
                turnOutput.p, turnOutput.i, turnOutput.d, turn);

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
    public boolean alignToAprilTag(Pose3D tagPose) {
        if (tagPose == null) {
            follower.setTeleOpDrive(0, 0, 0, true);
            telemetryM.debug("AlignToAprilTag", "No tags detected");
            driveController.reset();
            strafeController.reset();
            turnController.reset();
            return false;
        }

        // Error terms are now directly from the pose
        return alignToPose(tagPose.getPosition().x, tagPose.getPosition().y, tagPose.getPosition().z, 10.0);
    }
}
