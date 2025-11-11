package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {

    public enum IntakeUnitState {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    // If needed, make public methods to set this.
    private IntakeUnitState leftIntakeState;
    private IntakeUnitState rightIntakeState;

    private final DcMotor leftIntake;
    private final DcMotor rightIntake;

    public IntakeSubsystem(HardwareMap hardwareMap) {

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        leftIntakeState = IntakeUnitState.IDLE;
        rightIntakeState = IntakeUnitState.IDLE;
    }

    private double power = 0.96;

    public void runTeleOp(Gamepad gamepad) {
        // --- 1. Handle State Transitions (Controller Logic) ---
        // This logic is now split into two independent blocks.

        // Determine the state for the FRONT intake
        if (gamepad.left_trigger > 0) {
            leftIntakeState = IntakeUnitState.INTAKE;
        } else if (gamepad.left_bumper) {
            leftIntakeState = IntakeUnitState.OUTTAKE;
        } else {
            leftIntakeState = IntakeUnitState.IDLE;
        }

        // Determine the state for the BACK intake
        if (gamepad.right_trigger > 0) {
            rightIntakeState = IntakeUnitState.INTAKE;
        } else if (gamepad.right_bumper) {
            rightIntakeState = IntakeUnitState.OUTTAKE;
        } else {
            rightIntakeState = IntakeUnitState.IDLE;
        }


        // --- 2. Execute State Actions (Motor Logic) ---

        // Set power for the FRONT intake based on its state
        switch (leftIntakeState) {
            case IDLE:
                leftIntake.setPower(0);
                break;
            case INTAKE:
                leftIntake.setPower(power);
                break;
            case OUTTAKE:
                leftIntake.setPower(-power * 0.6);
                break;
        }

        // Set power for the BACK intake based on its state
        switch (rightIntakeState) {
            case IDLE:
                rightIntake.setPower(0);
                break;
            case INTAKE:
                rightIntake.setPower(power);
                break;
            case OUTTAKE:
                rightIntake.setPower(-power * 0.6);
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

    public void setRightIntake(double power) {
        leftIntake.setPower(power);
    }

    public void setLeftIntake(double power) {
        rightIntake.setPower(power);
    }

    public void stop() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

    public void sendAllTelemetry(Telemetry telemetry, boolean enableAll) {
        telemetry.addLine("\\ INTAKE SUBSYSTEM //");
        if (enableAll) {
            telemetry.addData("Intake Power", getIntakePower());
        }
    }

    public double getIntakePower() {
        return power;
    }
}
