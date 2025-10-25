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
    private final CRServo triggerServo;
    private final CRServo hoodServo;

    private final int SHOOTER_TICKS_PER_REV = 28;
    private final double MAX_RPM = 6000;
    private double STATIONARY_RPM = 3500;

    // Configurable in FTC Dashboard
    public static double kP = 0.015;
    public static double kD = 0.000;
    public static double RPM_TOLERANCE = 10;


    private final ElapsedTime triggerTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();
    private String shooterMessage;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        triggerServo = hardwareMap.get(CRServo.class, "triggerServo");
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        triggerTimer.reset();
        shooterTimer.reset();
    }

    // RPM PID
    private double targetRPM = 0;
    private double targetPower = 0;
    private double currentRPM = 0;

    private double prevError;
    public double shooterPID() {
        currentRPM = shooterMotor.getVelocity() / SHOOTER_TICKS_PER_REV * 60;
        double curError = targetRPM - currentRPM;
        if (Math.abs(curError) <= RPM_TOLERANCE) {
            curError = 0;
        }

        double p = curError * kP;
        double d = 0;
        if (shooterTimer.seconds() > 0)
            d = kD * (curError - prevError) / (shooterTimer.seconds());

        prevError = curError;
        shooterTimer.reset();

        return p + d;
    }

    public void runTeleOp(Gamepad gamepad) {

        // Control Logic
        if (gamepad.aWasPressed()) {
            triggerTimer.reset();
        } else if (gamepad.a) {
            pullTrigger();
            shooterMessage = "firing";
        } else {
            releaseTrigger();
            shooterMessage = "idle";
        }

        // Adjust
        if (gamepad.dpadUpWasPressed()) {
            STATIONARY_RPM += 100;
        } else if (gamepad.dpadDownWasPressed()) {
            STATIONARY_RPM -= 100;
        }


        if (gamepad.right_bumper) {
            targetRPM = STATIONARY_RPM;
            targetRPM = Math.max(0, Math.min(MAX_RPM, targetRPM));
            targetPower = shooterPID();
        } else if (getCurrentRPM() > 0) {
            targetRPM = 0;
            targetPower -= 0.05;
        }

        // final power limits.
        targetPower = Math.min(1.0, Math.max(0.0, targetPower));
        shooterMotor.setPower(targetPower);
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

    // PRIMITIVE METHODS

    public void pullTrigger() {
        triggerServo.setPower(1.0);
    }

    public void releaseTrigger() {
        triggerServo.setPower(0.0);
    }

    public double getTimerSeconds() {
        return triggerTimer.seconds();
    }

    // ACCESSOR
    public String shooterMessage() {
        return shooterMessage;
    }
    public double getStationaryRPM() {
        return STATIONARY_RPM;
    }
    public double getTargetRPM() {
        return targetRPM;
    }
    public double getCurrentRPM() {
        return currentRPM;
    }

}
