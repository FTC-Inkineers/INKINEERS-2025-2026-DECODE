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

    private final DcMotor backIntake;
    private final DcMotor frontIntake;

    public IntakeSubsystem(HardwareMap hardwareMap, boolean isBlueSide) {

        backIntake = hardwareMap.get(DcMotor.class, isBlueSide ? "leftIntake" : "rightIntake");
        frontIntake = hardwareMap.get(DcMotor.class, isBlueSide ? "rightIntake" : "leftIntake");

        backIntake.setDirection(isBlueSide ? DcMotor.Direction.REVERSE: DcMotor.Direction.FORWARD);
        frontIntake.setDirection(isBlueSide ? DcMotor.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);

        intakeState = IntakeState.DUAL_IDLE;
    }

    private double power = 1.0;

    public void runTeleOp(Gamepad gamepad) {
        // Control Logic
        // Left Side Intake
        double frontPower;
        if (gamepad.left_bumper) {
            frontPower = -power * 0.6;
        } else if (gamepad.left_trigger > 0) {
            frontPower = power;
        } else {
            frontPower = 0;
        }
        // Right Side Intake
        double backPower;
        if (gamepad.right_bumper) {
            backPower = -power * 0.6;
        } else if (gamepad.right_trigger > 0) {
            backPower = power;
        } else {
            backPower = 0;
        }

        backIntake.setPower(frontPower);
        frontIntake.setPower(backPower);

        // Adjust Power
        if (gamepad.rightBumperWasPressed()) {
            power += 0.05;
        } else if (gamepad.leftBumperWasPressed()) {
            power -= 0.05;
        }
    }

    public void setFrontIntake(double power) {
        backIntake.setPower(power);
    }

    public void setBackIntake(double power) {
        frontIntake.setPower(power);
    }

    public void stop() {
        backIntake.setPower(0);
        frontIntake.setPower(0);
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
