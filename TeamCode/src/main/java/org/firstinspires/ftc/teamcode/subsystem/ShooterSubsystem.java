package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/** @noinspection FieldCanBeLocal*/
@Config
public class ShooterSubsystem {
    private final DcMotorEx shooterMotor; // 6000 RPM YellowJacket Motor
    private final DcMotor triggerMotor;
    private final CRServo hoodServo;

    private final int SHOOTER_TICKS_PER_REV = 28;
    private final double MAX_RPM = 6000;
    private double STATIONARY_RPM_FAR = 3500;
    private double STATIONARY_RPM_CLOSE = 2800;

    public static double triggerPower = 0.92;

    // Configurable in FTC Dashboard
    public static double kF = 0.78/3514.0; // Motor Power / RPM | 0.78 / 3514 RPM
    public static double kP = 0.01;
    public static double kD = 0.0008;
    public static double RPM_TOLERANCE = 40;
    // Smoothing coefficient (alpha) for the Exponential Moving Average filter.
    // A lower value means more smoothing but more lag. Try a value between 0.1 and 0.3.
    public static double SMOOTHING_ALPHA = 0.1;


    private final ElapsedTime triggerTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private String shooterMessage;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        triggerMotor = hardwareMap.get(DcMotor.class, "triggerMotor");
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        triggerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        triggerTimer.reset();
        shooterTimer.reset();
    }

    // RPM PID
    private double targetRPM = 0;
    private double targetPower = 0;

    // Variable to hold the previously calculated, filtered RPM.
    private double lastFilteredRPM = 0.0;
    public double getCurrentRPM() {
        // 1. Calculate the raw, instantaneous RPM
        double rawRPM = shooterMotor.getVelocity() / SHOOTER_TICKS_PER_REV * 60;
        // 2. Apply the Exponential Moving Average (EMA) Filter
        double filteredRPM = (SMOOTHING_ALPHA * rawRPM) +
                ((1.0 - SMOOTHING_ALPHA) * lastFilteredRPM);
        // 3. Update the stored value for the next loop iteration
        lastFilteredRPM = filteredRPM;

        return filteredRPM;
    }

    private double prevError;
    private double currError;
    public double shooterPID() {
        currError = targetRPM - getCurrentRPM();
        if (Math.abs(currError) <= RPM_TOLERANCE) {
            currError = 0;
        }

        double f = kF * targetRPM;
        double p = currError * kP;
        double d = 0;
        if (shooterTimer.seconds() > 0)
            d = kD * (currError - prevError) / (shooterTimer.seconds());

        prevError = currError;
        shooterTimer.reset();

        return f + p + d;
    }

    public void runTeleOp(Gamepad gamepad) {

        // Control Logic
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

        // Adjust
        if (gamepad.dpadUpWasPressed()) {
            STATIONARY_RPM_FAR += 100;
        } else if (gamepad.dpadDownWasPressed()) {
            STATIONARY_RPM_FAR -= 100;
        }


        if (gamepad.right_bumper || gamepad.left_bumper) {
            if (gamepad.right_bumper)
                targetRPM = STATIONARY_RPM_FAR;
            else if (gamepad.left_bumper)
                targetRPM = STATIONARY_RPM_CLOSE;

            targetRPM = Math.max(0, Math.min(MAX_RPM, targetRPM));
            targetPower = shooterPID();
        } else if (getCurrentRPM() > 0) {
            targetRPM = 0;
            targetPower -= 0.025;
        }

        // final power limits.
        targetPower = Math.min(1.0, Math.max(0.0, targetPower));
        shooterMotor.setPower(targetPower);
    }

    public void runAuto() {

        targetPower = Math.min(1.0, Math.max(0.0, targetPower));
        shooterMotor.setPower(targetPower);
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    public void autoShoot() {

    }

    public boolean isIdle() {
        return targetRPM <= 10;
    }

    public boolean isActive() {
        return targetRPM > 1000 || Math.abs(currError) > 100;
    }

    public void enableAllTelemetry(OpMode opMode, boolean enableAll) {
        opMode.telemetry.addLine("\\ SHOOTER SUBSYSTEM //");
        if (enableAll) {
            opMode.telemetry.addData("Shooter Message", shooterMessage());
            opMode.telemetry.addData("Trigger Time", getTimerSeconds());
            opMode.telemetry.addLine();
            opMode.telemetry.addData("Stationary RPM", getStationaryRPM());
        }
        opMode.telemetry.addData("Current Target RPM:", getTargetRPM());
        opMode.telemetry.addData("Current RPM:", getCurrentRPM());
        opMode.telemetry.addData("Shooter PID", targetPower);
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
