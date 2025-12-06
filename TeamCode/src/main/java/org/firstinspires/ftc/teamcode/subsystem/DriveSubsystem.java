package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class DriveSubsystem {
    // Tunable proportional gains for AprilTag alignment
    public static double kP_drive = 0.01, kD_drive = 0.00;       // Forward/backward
    public static double kP_strafe = 0.01, kD_strafe = 0.00;     // Left/right
    public static double kP_turn = 0.01, kD_turn = 0.00;         // Turn

    public static double maxTurnPower = 0.45;

    private final FPIDController driveController = new FPIDController.Builder(kP_drive).withD(kD_drive).build();
    private final FPIDController strafeController = new FPIDController.Builder(kP_strafe).withD(kD_strafe).build();
    private final FPIDController turnController = new FPIDController.Builder(kP_turn).withD(kD_turn).build();
    
    // PEDRO PATHING
    public final Follower follower;
    public static Pose startingPose;
    private final TelemetryManager telemetryM;

    // Fields for Hold Position Logic
    private Pose holdPositionTarget;

    // TELEOP
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private final VisionSubsystem vision;
    private final CRServo windmill;

    InputRamper forwardRamper, strafeRamper, turnRamper;
    
    public enum DriveState {
        MANUAL,
        MANUAL_AIM_ASSIST,
        HOLD_POSITION
    }
    
    private DriveState driveState;

    public DriveSubsystem(HardwareMap hardwareMap, VisionSubsystem visionSubsystem, boolean isBlueSide) {

        follower = Constants.createFollower(hardwareMap, isBlueSide);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        vision = visionSubsystem;
        windmill = hardwareMap.get(CRServo.class, "windmill");
    }

    public void initTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        forwardRamper = new InputRamper();
        strafeRamper = new InputRamper();
        turnRamper = new InputRamper();
        
        driveState = DriveState.MANUAL;
        lastManualState = DriveState.MANUAL;
    }

    public void initAuto(Pose startPose) {
        follower.setStartingPose(startPose == null ? new Pose() : startPose);
        follower.update();
    }
    
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    private void updateSystems() {
        follower.update();
        telemetryM.update();

        // Update PID Gains Live
        driveController.setGains(0, kP_drive, 0, kD_drive);
        strafeController.setGains(0, kP_strafe, 0, kD_strafe);
        turnController.setGains(0, kP_turn, 0, kD_turn);
    }

    public void runTeleOp() {
        runTeleOp(false);
    }

    public void runTeleOp(boolean shooterIsActive) {
        updateSystems();
        handleStateTransitions();

        switch (driveState) {
            case MANUAL:
                handleManualDrive(false, shooterIsActive);
                break;
            case MANUAL_AIM_ASSIST:
                handleManualDrive(true, shooterIsActive);
                break;
            case HOLD_POSITION:
                holdPosition();
                break;
        }

        // Reset Pose (Global command)
        if (gamepad1.backWasPressed()) {
            follower.setPose(new Pose(0, 0, 0));
        }
    }

    DriveState lastManualState;
    private void handleStateTransitions() {
        // HOLD_POSITION has the highest priority. Pressing Y enters this state.
        if (gamepad1.y) {
            if (gamepad1.yWasPressed()) {
                if (driveState == DriveState.MANUAL) {
                    driveState = DriveState.MANUAL_AIM_ASSIST;
                    lastManualState = DriveState.MANUAL_AIM_ASSIST;
                    telemetryM.debug("Drive State", "MANUAL -> AIM_ASSIST");
                } else {
                    driveState = DriveState.MANUAL;
                    lastManualState = DriveState.MANUAL;
                    telemetryM.debug("Drive State", "AIM_ASSIST -> MANUAL");
                }
            } else {
                // Entering HOLD_POSITION for the first time
                driveState = DriveState.HOLD_POSITION;
                holdPositionTarget = follower.getPose();
                telemetryM.debug("Drive State", "ENTERING HOLD_POSITION");
            }
        }
        // If Y is released, go back to MANUAL.
        else if (driveState == DriveState.HOLD_POSITION) {
            driveState = lastManualState;
            telemetryM.debug("Drive State", "EXITING HOLD_POSITION -> MANUAL");
        }
    }

    private void handleManualDrive(boolean withAimAssist, boolean shooterIsActive) {
        double leftYInput = forwardRamper.rampInput(gamepad1.left_stick_y);
        double leftXInput = strafeRamper.rampInput(gamepad1.left_stick_x);
        double rightXInput;

        if (withAimAssist && vision.isTargetVisible()) {
            rightXInput = getAprilTagTurnCommand();
        } else {
            rightXInput = turnRamper.rampInput(gamepad1.right_stick_x) * (shooterIsActive ? 0.4 : 0.8);
        }

        // Field-centric control for manual driving
        follower.setTeleOpDrive(-leftYInput, -leftXInput, -rightXInput, false);

        // Windmill
        Vector windVector = new Vector(-leftYInput, -leftXInput);
        windVector.times(-follower.getHeading());

        windmill.setPower(windVector.getXComponent()+rightXInput);
    }

    public void sendAllTelemetry(Telemetry telemetry, boolean enableAll) {
        telemetry.addLine("\\ DRIVE //");
        if (enableAll) {
            telemetry.addData("position", follower.getPose());
            telemetry.addData("velocity", follower.getVelocity());
        }
        if (driveState != null)
            telemetry.addData("Drive State", driveState.toString());
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

    // TODO: use tag Y angle to add an offset
    private boolean lockedOn = false;
    private double getAprilTagTurnCommand() {
        LLResultTypes.FiducialResult targetTag = vision.getTargetTag();

        if (targetTag == null) {
            telemetryM.debug("Lock On", "No tags detected");
            turnController.reset();
            return 0.0;
        }

        boolean farShot = targetTag.getTargetYDegrees() < -0.5;

        // Error is yaw.
        double currentError = targetTag.getTargetXDegrees();
        lockedOn = Math.abs(currentError) < 3.0;
        FPIDOutput turnOutput = turnController.calculate(currentError);
        double turnPower = turnOutput.total;

        // Clamp speed
        turnPower = Math.max(-maxTurnPower, Math.min(maxTurnPower, turnPower));

        telemetryM.debug("LockOn", "yawErr: %.2f", currentError);
        telemetryM.debug("LockOnPID", "P: %.2f, I: %.2f, D: %.2f, Total: %.2f",
                turnOutput.p, turnOutput.i, turnOutput.d, turnPower);

        return turnPower;
    }

    private void holdPosition() {
        if (holdPositionTarget == null) return;

        Pose currentPose = follower.getPose();

        // 1. Calculate the field-centric error (how far off we are on the field grid)
        Pose fieldError = currentPose.minus(holdPositionTarget);

        // 2. Rotate the field-centric error into a robot-centric error using the .rotate() method.
        // We rotate by the *negative* of the robot's current heading.
        // We set rotateHeading to 'false' because we want the raw heading error, not a rotated one.
        Pose robotCentricError = fieldError.rotate(-currentPose.getHeading(), false);

        // 3. Extract the errors from the new robot-centric pose.
        double xError = robotCentricError.getX(); // This is now the strafe error
        double yError = robotCentricError.getY(); // This is now the forward/backward error
        double headingError = fieldError.getHeading();

        // Calculate PID outputs
        FPIDOutput driveOutput = driveController.calculate(yError); // Forward/backward error
        FPIDOutput strafeOutput = strafeController.calculate(xError); // Left/right error
        FPIDOutput turnOutput = turnController.calculate(headingError); // Heading error

        double drive = driveOutput.total;
        double strafe = strafeOutput.total;
        double turn = turnOutput.total;

        // Clamp motor powers to reasonable limits
        drive = Math.max(-1.0, Math.min(1.0, drive));
        strafe = Math.max(-1.0, Math.min(1.0, strafe));
        turn = Math.max(-0.8, Math.min(0.8, turn));

        // Send commands to the drivetrain. Note the sign changes to match motor directions.
        follower.setTeleOpDrive(-drive, -strafe, -turn, true); // Use Robot-Centric for direct control

        // DETAILED TELEMETRY
        telemetryM.debug("HoldPosition", "Target: " + holdPositionTarget.toString());
        telemetryM.debug("HoldPosition", "Current: " + currentPose);
        telemetryM.debug("HoldErrors", "xErr: %.2f, yErr: %.2f, hErr: %.2f", xError, yError, headingError);
        telemetryM.debug("DrivePID", "P: %.2f, I: %.2f, D: %.2f, Total: %.2f",
                driveOutput.p, driveOutput.i, driveOutput.d, drive);
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public boolean getLockedOn() {
        return lockedOn;
    }
}
