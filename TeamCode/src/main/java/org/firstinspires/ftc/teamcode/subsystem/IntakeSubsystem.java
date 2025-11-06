package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    public enum IntakeUnitState {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    // If needed, make public methods to set this.
    private IntakeUnitState frontIntakeState;
    private IntakeUnitState backIntakeState;

    private final DcMotor backIntake;
    private final DcMotor frontIntake;

    public IntakeSubsystem(HardwareMap hardwareMap) {

        backIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        frontIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        backIntake.setDirection(DcMotor.Direction.REVERSE);
        frontIntake.setDirection(DcMotor.Direction.FORWARD);

        frontIntakeState = IntakeUnitState.IDLE;
        backIntakeState = IntakeUnitState.IDLE;
    }

    private double power = 1.0;

    public void runTeleOp(Gamepad gamepad) {
        // --- 1. Handle State Transitions (Controller Logic) ---
        // This logic is now split into two independent blocks.

        // Determine the state for the FRONT intake
        if (gamepad.left_trigger > 0) {
            frontIntakeState = IntakeUnitState.INTAKE;
        } else if (gamepad.left_bumper) {
            frontIntakeState = IntakeUnitState.OUTTAKE;
        } else {
            frontIntakeState = IntakeUnitState.IDLE;
        }

        // Determine the state for the BACK intake
        if (gamepad.right_trigger > 0) {
            backIntakeState = IntakeUnitState.INTAKE;
        } else if (gamepad.right_bumper) {
            backIntakeState = IntakeUnitState.OUTTAKE;
        } else {
            backIntakeState = IntakeUnitState.IDLE;
        }


        // --- 2. Execute State Actions (Motor Logic) ---

        // Set power for the FRONT intake based on its state
        switch (frontIntakeState) {
            case IDLE:
                frontIntake.setPower(0);
                break;
            case INTAKE:
                frontIntake.setPower(power);
                break;
            case OUTTAKE:
                frontIntake.setPower(-power * 0.6);
                break;
        }

        // Set power for the BACK intake based on its state
        switch (backIntakeState) {
            case IDLE:
                backIntake.setPower(0);
                break;
            case INTAKE:
                backIntake.setPower(power);
                break;
            case OUTTAKE:
                backIntake.setPower(-power * 0.6);
                break;
        }

        // Power adjustment logic can remain the same
        if (gamepad.dpad_up) {
            power += 0.05;
        } else if (gamepad.dpad_down) {
            power -= 0.05;
        }
        power = Math.max(0.0, Math.min(1.0, power));
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
