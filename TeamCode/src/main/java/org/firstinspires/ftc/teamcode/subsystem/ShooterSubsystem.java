package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSubsystem {
    private final DcMotor shooterMotor;
    private final Servo triggerServo;
    private final CRServo hoodServo;

    private final ElapsedTime triggerTimer = new ElapsedTime();
    private String shooterMessage;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        triggerServo = hardwareMap.get(Servo.class, "triggerServo");
        hoodServo = hardwareMap.get(CRServo.class, "hoodServo");

        triggerTimer.reset();
    }

    public void runTeleOp(Gamepad gamepad) {

        // Control Logic
        if (gamepad.rightBumperWasPressed()) {
            fire();
        }
        if (gamepad.right_trigger > 0) {
            spinUp();
        } else if (getShooterPower() >= 1.0) {
            spinRelease();
        }

        if (triggerTimer.seconds() <= 0.4) {
            pullTrigger();
        } else {
            releaseTrigger();
            shooterMessage = "idle";
        }
    }

    public String shooterMessage() {
        return shooterMessage;
    }

    // PRIMITIVE METHODS
    public void spinUp() {
        shooterMotor.setPower(1.0);
    }

    public void spinRelease() {
        shooterMotor.setPower(0.0);
    }

    public void pullTrigger() {
        triggerServo.setPosition(1.0);
    }

    public void releaseTrigger() {
        triggerServo.setPosition(0.0);
    }


    // SHOOTER TIMER
    public void fire() {
        triggerTimer.reset();
        shooterMessage = "fired!";
    }

    public double getTimerSeconds() {
        return triggerTimer.seconds();
    }

    // ACCESSOR
    public double getShooterPower() {
        return shooterMotor.getPower();
    }
}
