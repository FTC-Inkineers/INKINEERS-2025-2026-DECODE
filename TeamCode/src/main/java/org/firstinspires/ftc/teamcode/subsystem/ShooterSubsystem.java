package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_FLYWHEEL_RPM;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** @noinspection FieldCanBeLocal*/
@Config
public class ShooterSubsystem {
    // Hardware
    private final DcMotorEx shooterMotor; // 6000 RPM YellowJacket Motor
    private final DcMotor triggerMotor;
    private final Servo hoodServo;

    // Constants
    private final int SHOOTER_TICKS_PER_REV = 28;
    private final double MAX_RPM = MAX_FLYWHEEL_RPM;
    private double STATIONARY_RPM_FAR = 3200;
    private final double STATIONARY_RPM_CLOSE = 2800;

    private final double HOOD_MAX_EXTEND = 1;
    private final double HOOD_MAX_RETRACT = 0.5;

    public static double triggerPower = 0.92;

    // Tuning (Dashboard)
    public static double kF = 0.78 / 3514.0; // Motor Power / RPM | 0.78 / 3514 RPM
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0008;
    public static double SMOOTHING_ALPHA = 0.1; // Try a value between 0.1 and 0.3.
    public static double RPM_TOLERANCE = 100; // Allowable error to be considered "READY"

    private final FPIDController shooterController = new FPIDController.Builder(kP).withF(kF).withD(kD).build();

    public enum ShooterState {
        IDLE,           // Motor is off (0 power)
        RAMPING_UP,     // Target set, PID active, error is large
        READY,          // Target set, PID active, error is small (within tolerance)
        RAMPING_DOWN    // Target is 0, but motor is still spinning (coasting down)
    }

    private ShooterState currentState = ShooterState.IDLE;
    private String triggerMessage = "idle";

    private final ElapsedTime triggerTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();

    private double targetRPM = 0;
    private double targetPower = 0;
    private double lastFilteredRPM = 0.0;
    private double error;
    private double hoodPosition = HOOD_MAX_RETRACT;

    // External Subsystems
    private VisionSubsystem vision;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        triggerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        triggerTimer.reset();
        shooterTimer.reset();
    }

    public void initTeleOp(VisionSubsystem vision) {
        this.vision = vision;
    }

    public void updateShooterPhysics() {
        double currentRPM = getCurrentRPM();
        error = targetRPM - currentRPM;

        // Calculate power using the controller. Pass both the error and the target (for kF).
        FPIDOutput output = shooterController.calculate(error, targetRPM);
        targetPower = output.total;
    }

    public void runTeleOp(Gamepad gamepad) {
        // 1. Update Tuning
        shooterController.setGains(kF, kP, kI, kD);

        // 2. Handle Inputs (Determine Desired RPM)
        runShooter(gamepad);
        runTrigger(gamepad);
        runHood(gamepad);

        // 3. Calculate Physics
        updateShooterPhysics();

        // 4. Update State Logic
        updateState();

        // 5. Apply Power based on State
        updateMotorPower();
    }

    private void updateState() {
        double currentRPM = getCurrentRPM();

        switch (currentState) {
            case IDLE:
                if (targetRPM > 0) {
                    currentState = ShooterState.RAMPING_UP;
                }
                break;

            case RAMPING_UP:
                if (targetRPM == 0) {
                    currentState = ShooterState.RAMPING_DOWN;
                } else if (Math.abs(error) < RPM_TOLERANCE) {
                    currentState = ShooterState.READY;
                }
                break;

            case READY:
                if (targetRPM == 0) {
                    currentState = ShooterState.RAMPING_DOWN;
                } else if (Math.abs(error) > RPM_TOLERANCE) {
                    currentState = ShooterState.RAMPING_UP; // Lost speed, ramp back up
                }
                break;

            case RAMPING_DOWN:
                if (targetRPM > 0) {
                    currentState = ShooterState.RAMPING_UP;
                } else if (currentRPM < 50) { // Threshold for "stopped"
                    currentState = ShooterState.IDLE;
                }
                break;
        }
    }

    /**
     * Applies actual motor power based on the current State
     */
    private void updateMotorPower() {
        double finalPower = 0.0;

        switch (currentState) {
            case IDLE:
                finalPower = 0;
                shooterController.reset(); // Clear integral windup
                break;

            case RAMPING_UP:
            case READY:
                // Use the PID calculated power
                finalPower = targetPower;
                break;

            case RAMPING_DOWN:
                // Custom behavior: Gradual slowdown
                finalPower -= 0.025;
                break;
        }

        // Safety Clamp
        finalPower = Math.min(1.0, Math.max(0.0, finalPower));
        shooterMotor.setPower(finalPower);
    }

    private void runShooter(Gamepad gamepad) {
        // RPM Adjustment
        if (gamepad.dpadUpWasPressed()) {
            STATIONARY_RPM_FAR += 100;
        } else if (gamepad.dpadDownWasPressed()) {
            STATIONARY_RPM_FAR -= 100;
        }

        // Determine if we want to shoot
        boolean wantToShoot = (gamepad.right_trigger > 0 || gamepad.left_trigger > 0);

        if (wantToShoot) {
            LLResultTypes.FiducialResult targetTag = vision.getTargetTag();

            if (targetTag != null) {
                // Auto-select RPM based on vision
                targetRPM = (targetTag.getTargetYDegrees() < -0.5) ? STATIONARY_RPM_FAR : STATIONARY_RPM_CLOSE;
            } else if (gamepad.right_trigger > 0) {
                targetRPM = STATIONARY_RPM_FAR;
            } else {
                targetRPM = STATIONARY_RPM_CLOSE;
            }
            // Safety Clamp
            targetRPM = Math.max(0, Math.min(MAX_RPM, targetRPM));
        } else {
            // User released trigger, we want 0 RPM
            targetRPM = 0;
        }
    }

    public void runTrigger(Gamepad gamepad) {
        if (gamepad.aWasPressed()) {
            triggerTimer.reset();
        } else if (gamepad.b) {
            reverseTrigger();
            triggerMessage = "reversing";
        } else if (gamepad.a) {
            pullTrigger();
            triggerMessage = "firing";
        } else {
            releaseTrigger();
            triggerMessage = "idle";
        }
    }

    public void runHood(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            hoodPosition += 0.1;
        } else if (gamepad.left_bumper) {
            hoodPosition -= 0.1;
        }
        hoodPosition = Math.max(HOOD_MAX_RETRACT, Math.min(HOOD_MAX_EXTEND, hoodPosition));
        hoodServo.setPosition(hoodPosition);
    }

    // --- UTILITY METHODS ---

    public double getCurrentRPM() {
        double rawRPM = shooterMotor.getVelocity() / SHOOTER_TICKS_PER_REV * 60;
        // Apply EMA filter
        double filteredRPM = (SMOOTHING_ALPHA * rawRPM) + ((1.0 - SMOOTHING_ALPHA) * lastFilteredRPM);
        lastFilteredRPM = filteredRPM;
        return filteredRPM;
    }

    // --- PUBLIC HELPERS (For Auto/OpMode) ---

    public boolean isReady() {
        return currentState == ShooterState.READY;
    }

    public ShooterState getState() {
        return currentState;
    }

    // --- TELEMETRY ---

    public void sendAllTelemetry(Telemetry telemetry, boolean enableAll) {
        telemetry.addLine("\\ SHOOTER SUBSYSTEM //");
        telemetry.addData("State", currentState.toString()); // NEW: Shows state

        if (enableAll) {
            telemetry.addData("Trigger Msg", triggerMessage);
            telemetry.addData("Stationary RPM", getStationaryRPM_Far());
        }
        telemetry.addData("Target RPM:", targetRPM);
        telemetry.addData("Current RPM:", getCurrentRPM());
        telemetry.addData("Error:", error);
        telemetry.addData("Motor Power", shooterMotor.getPower());
    }

    // --- PRIMITIVES ---

    public void pullTrigger() {
        triggerMotor.setPower(triggerPower);
    }
    public void reverseTrigger() {
        triggerMotor.setPower(-triggerPower);
    }
    public void releaseTrigger() {
        triggerMotor.setPower(0.0);
    }

    public double getStationaryRPM_Far() {
        return STATIONARY_RPM_FAR;
    }

    public double getStationaryRPM_Close() {
        return STATIONARY_RPM_CLOSE;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isActive() {
        return targetRPM > 1000 || Math.abs(error) < 100;
    }

    // --- AUTONOMOUS ---
    // --- AUTONOMOUS ---
    // --- AUTONOMOUS ---

    public void runAuto() {
        updateState();
        updateMotorPower();
    }

    public void windDown() {
        targetPower = 0.0;
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    public boolean isIdle() {
        return targetRPM <= 10;
    }

    // Kf Testing

    public double testPower = 0.5;
    public double changeFactor = 0.05;

    public void runKfTester(Gamepad gamepad) {
        // 0.78 | 3514 RPM
        targetRPM = 3500;
        if (gamepad.dpadUpWasPressed()) {
            testPower += changeFactor;
        } else if (gamepad.dpadDownWasPressed()) {
            testPower -= changeFactor;
        }

        if (gamepad.right_bumper) {
            changeFactor = 0.05;
        } else if (gamepad.left_bumper) {
            changeFactor = 0.01;
        }

        shooterMotor.setPower(testPower);
    }
}
