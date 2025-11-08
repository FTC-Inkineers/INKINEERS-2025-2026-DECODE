package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
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
    private final DcMotorEx shooterMotor; // 6000 RPM YellowJacket Motor
    private final DcMotor triggerMotor;
    private final Servo hoodServo;

    private final int SHOOTER_TICKS_PER_REV = 28;
    private final double MAX_RPM = 6000;
    private double STATIONARY_RPM_FAR = 3500;
    private final double STATIONARY_RPM_CLOSE = 2800;

    private final double HOOD_MAX_EXTEND = 1;
    private final double HOOD_MAX_RETRACT = 0.5;

    private final FPIDController shooterController;

    public static double triggerPower = 0.92;

    // Configurable in FTC Dashboard
    public static double kF = 0.78/3514.0; // Motor Power / RPM | 0.78 / 3514 RPM
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0008;
    public static double RPM_TOLERANCE = 40;
    // A lower value means more smoothing but more lag. Try a value between 0.1 and 0.3.
    public static double SMOOTHING_ALPHA = 0.1;

    private final ElapsedTime triggerTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private String shooterMessage;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        triggerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        triggerTimer.reset();
        shooterTimer.reset();

        shooterController = new FPIDController.Builder(kP).withF(kF).withD(kD).build();
    }

    // RPM PID
    private double targetRPM = 0;
    private double targetPower = 0;

    // Variable to hold the previously calculated, filtered RPM.
    private double lastFilteredRPM = 0.0;
    public double getCurrentRPM() {
        double rawRPM = shooterMotor.getVelocity() / SHOOTER_TICKS_PER_REV * 60;

        // Apply the Exponential Moving Average (EMA) Filter
        double filteredRPM = (SMOOTHING_ALPHA * rawRPM) + ((1.0 - SMOOTHING_ALPHA) * lastFilteredRPM);

        lastFilteredRPM = filteredRPM;

        return filteredRPM;
    }

    private double error;
    public void updateShooterPower() {
        double currentRPM = getCurrentRPM();
        error = targetRPM - currentRPM;

        if (Math.abs(error) <= RPM_TOLERANCE) {
            // You can optionally treat small errors as zero, but often it's better to let PID handle it.
            // For now, we'll keep it simple.
        }

        // Calculate power using the controller. Pass both the error and the target (for kF).
        FPIDOutput output = shooterController.calculate(error, targetRPM);
        targetPower = output.total;
    }

    public void runTeleOp(Gamepad gamepad) {
        // Ensure changes from FTC Dashboard are always applied.
        shooterController.setGains(kF, kP, kI, kD);

        runTrigger(gamepad);
        runShooter(gamepad);
        runHood(gamepad);
    }

    public void runTrigger(Gamepad gamepad) {
        // Trigger Control
        if (gamepad.aWasPressed()) {
            triggerTimer.reset();
        } else if (gamepad.b) {
            reverseTrigger();
            shooterMessage = "reversing";
        } else if (gamepad.a) {
            pullTrigger();
            shooterMessage = "firing";
        } else {
            releaseTrigger();
            shooterMessage = "idle";
        }
    }

    public void runShooter(Gamepad gamepad) {
        // Adjust Target RPM
        if (gamepad.dpadUpWasPressed()) {
            STATIONARY_RPM_FAR += 100;
        } else if (gamepad.dpadDownWasPressed()) {
            STATIONARY_RPM_FAR -= 100;
        }

        // Shooter (Flywheel) Control
        if (gamepad.right_trigger > 0 || gamepad.left_trigger > 0) {
            if (gamepad.right_trigger > 0)
                targetRPM = STATIONARY_RPM_FAR;
            else if (gamepad.left_trigger > 0)
                targetRPM = STATIONARY_RPM_CLOSE;

            targetRPM = Math.max(0, Math.min(MAX_RPM, targetRPM));
            updateShooterPower();
        } else if (getCurrentRPM() > 0) {
            targetRPM = 0;
            targetPower -= 0.025;
            shooterController.reset();
        } else {
            targetRPM = 0;
            targetPower = 0;
        }

        targetPower = Math.min(1.0, Math.max(0.0, targetPower));
        shooterMotor.setPower(targetPower);
    }

    public void runHood(Gamepad gamepad) {
        // Hood Control
        if (gamepad.right_bumper) {
            hoodPosition += 0.1;
        } else if (gamepad.left_bumper) {
            hoodPosition -= 0.1;
        }
        hoodPosition = Math.max(HOOD_MAX_RETRACT, Math.min(HOOD_MAX_EXTEND, hoodPosition));
        hoodServo.setPosition(hoodPosition);
    }

    private double hoodPosition = HOOD_MAX_RETRACT;

    // --- AUTONOMOUS ---
    // --- AUTONOMOUS ---
    // --- AUTONOMOUS ---

    public void runAuto() {
        targetPower = Math.min(1.0, Math.max(0.0, targetPower));
        shooterMotor.setPower(targetPower);
    }

    boolean init = false;
    public void autoShoot(IntakeSubsystem intake) {
        if (!init) {
            init = true;
        }
        targetRPM = STATIONARY_RPM_FAR;
        intake.setFrontIntake(1);
        intake.setBackIntake(0.4);
        updateShooterPower();
    }

    public boolean isIdle() {
        return targetRPM <= 10;
    }

    public boolean isActive() {
        return targetRPM > 1000 || Math.abs(error) > 100;
    }

    public void enableAllTelemetry(Telemetry telemetry, boolean enableAll) {
        telemetry.addLine("\\ SHOOTER SUBSYSTEM //");
        if (enableAll) {
            telemetry.addData("Shooter Message", shooterMessage());
            telemetry.addData("Trigger Time", getTimerSeconds());
            telemetry.addLine();
            telemetry.addData("Stationary RPM", getStationaryRPM());
        }
        telemetry.addData("Current Target RPM:", getTargetRPM());
        telemetry.addData("Current RPM:", getCurrentRPM());
        telemetry.addData("Current Error:", error);
        telemetry.addData("Shooter PID", targetPower);
    }

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

    // PRIMITIVE METHODS

    public void pullTrigger() {
        triggerMotor.setPower(triggerPower);
    }

    public void reverseTrigger() {
        triggerMotor.setPower(-triggerPower);
    }

    public void releaseTrigger() {
        triggerMotor.setPower(0.0);
    }

    public double getTimerSeconds() {
        return triggerTimer.seconds();
    }

    // ACCESSOR
    public String shooterMessage() {
        return shooterMessage;
    }
    public double getStationaryRPM() {
        return STATIONARY_RPM_FAR;
    }
    public double getTargetRPM() {
        return targetRPM;
    }
}
