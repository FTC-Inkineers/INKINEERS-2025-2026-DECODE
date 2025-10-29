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

    private final DcMotor frontIntake;
    private final DcMotor backIntake;

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isBlueSide) {

        frontIntake = hardwareMap.get(DcMotor.class, isBlueSide ? "leftIntake" : "rightIntake");
        backIntake = hardwareMap.get(DcMotor.class, isBlueSide ? "rightIntake" : "leftIntake");

        frontIntake.setDirection(isBlueSide ? DcMotor.Direction.REVERSE: DcMotor.Direction.FORWARD);
        backIntake.setDirection(isBlueSide ? DcMotor.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

        intakeState = IntakeState.DUAL_IDLE;
    }

    private double power = 1.0;

    public void runTeleOp(Gamepad gamepad) {
        // Control Logic
        // Front Intake
        double frontPower;
        if (gamepad.right_bumper) {
            frontPower = -power;
        } else if (gamepad.right_trigger > 0) {
            frontPower = power;
        } else {
            frontPower = 0;
        }
        // Back Intake
        double backPower;
        if (gamepad.left_bumper) {
            backPower = -power;
        } else if (gamepad.left_trigger > 0) {
            backPower = power;
        } else {
            backPower = 0;
        }

        frontIntake.setPower(frontPower);
        backIntake.setPower(backPower);

        // Adjust Power
        if (gamepad.rightBumperWasPressed()) {
            power += 0.05;
        } else if (gamepad.leftBumperWasPressed()) {
            power -= 0.05;
        }
    }

    public void enableAllTelemetry(OpMode opMode, boolean enableAll) {
        opMode.telemetry.addLine("\\ INTAKE SUBSYSTEM //");
        if (enableAll) {
            opMode.telemetry.addData("Intake Power", getIntakePower());
        }
    }

    public double getIntakePower() {
        return power;
    }
}
