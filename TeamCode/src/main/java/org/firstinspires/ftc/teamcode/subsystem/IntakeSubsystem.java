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

    @SuppressWarnings("unused")
    public void setPower(double power) {
        this.power = power;
    }

    public void runTeleOp(Gamepad gamepad) {
        // --- 1. Handle State Transitions (Controller Logic) ---
        // This logic is now split into two independent blocks.

        // Determine the state for the LEFT intake
        if (gamepad.left_trigger > 0) {
            leftIntakeState = IntakeUnitState.INTAKE;
        } else if (gamepad.left_bumper) {
            leftIntakeState = IntakeUnitState.OUTTAKE;
        } else {
            leftIntakeState = IntakeUnitState.IDLE;
        }

        // Determine the state for the RIGHT intake
        if (gamepad.right_trigger > 0) {
            rightIntakeState = IntakeUnitState.INTAKE;
        } else if (gamepad.right_bumper) {
            rightIntakeState = IntakeUnitState.OUTTAKE;
        } else {
            rightIntakeState = IntakeUnitState.IDLE;
        }


        // --- 2. Execute State Actions (Motor Logic) ---

        // Set power for the LEFT intake based on its state
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

        // Set power for the RIGHT intake based on its state
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

    public void runAuto() {
        // Set power for the LEFT intake based on its state
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

        // Set power for the RIGHT intake based on its state
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
    }

    public enum IntakeSide {
        LEFT,
        RIGHT,
        BOTH
    }

    public void setIntake(IntakeSide side, IntakeUnitState state) {
        if (side == IntakeSide.LEFT || side == IntakeSide.BOTH) {
            this.leftIntakeState = state;
        }
        if (side == IntakeSide.RIGHT || side == IntakeSide.BOTH) {
            this.rightIntakeState = state;
        }
    }

    public void stop() {
        leftIntakeState = IntakeUnitState.IDLE;
        rightIntakeState = IntakeUnitState.IDLE;
    }

    public void sendAllTelemetry(Telemetry telemetry, boolean enableAll) {
        if (enableAll) {
            telemetry.addLine("\\ INTAKE SUBSYSTEM //");
            telemetry.addData("Intake Power", getIntakePower());
        }
    }

    public double getIntakePower() {
        return power;
    }
}
