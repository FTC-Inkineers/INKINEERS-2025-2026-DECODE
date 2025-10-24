package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        frontIntake.setDirection(blueSide ? DcMotor.Direction.FORWARD: DcMotor.Direction.REVERSE);
        backIntake.setDirection(blueSide ? DcMotor.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        intakeState = IntakeState.DUAL_IDLE;
    }

    private double power = 1.0;
    public void runTeleOp(Gamepad gamepad) {
        // Control Logic
        // RIGHT
        if (gamepad.right_trigger > 0 && gamepad.left_trigger > 0) {
            frontIntake.setPower(power);
            backIntake.setPower(power);
        } else if (gamepad.right_trigger > 0) {
            frontIntake.setPower(power * (gamepad.b ? -1 : 1));
        } else if (gamepad.left_trigger > 0) {
            backIntake.setPower(power * (gamepad.b ? -1 : 1));
        } else {
            frontIntake.setPower(0);
            backIntake.setPower(0);
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
        frontIntake.setPower(power);
    }

    public double getIntakePower() {
        return power;
    }
}
