package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    public enum IntakeState {
        DUAL_IDLE
    }

    IntakeState intakeState;

    private final boolean blueSide;

    private final DcMotor frontIntake;
    private final DcMotor backIntake;

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isBlueSide) {
        blueSide = isBlueSide;

        frontIntake = hardwareMap.get(DcMotor.class, isBlueSide ? "rightIntake" : "leftIntake");
        backIntake = hardwareMap.get(DcMotor.class, isBlueSide ? "leftIntake" : "rightIntake");

        backIntake.setDirection(DcMotor.Direction.REVERSE);

        intakeState = IntakeState.DUAL_IDLE;
    }

    private double power = 0.6;
    public void runTeleOp(Gamepad gamepad) {
        // Control Logic
        // RIGHT
        if (gamepad.right_trigger > 0) {
            backIntake.setPower(1);
        } else {
            backIntake.setPower(0);
        }
        // LEFT
        if (gamepad.left_trigger > 0) {
            frontIntake.setPower(1);
        } else {
            frontIntake.setPower(0);
        }
        // Adjust Power
        if (gamepad.rightBumperWasPressed()) {
            power += 0.05;
        } else if (gamepad.leftBumperWasPressed()) {
            power -= 0.05;
        }
    }

    public void enableAllTelemetry(OpMode opMode) {
        opMode.telemetry.addLine("\\ INTAKE SUBSYSTEM //");
        opMode.telemetry.addData("Intake Power", getIntakePower());
    }

    public void spinLeftIntake() {
        frontIntake.setPower(1.0);
    }

    public double getIntakePower() {
        return power;
    }
}
