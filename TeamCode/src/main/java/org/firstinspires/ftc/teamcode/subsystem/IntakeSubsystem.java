package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    public enum IntakeState {
        DUAL_IDLE
    }

    IntakeState intakeState;

    private final DcMotor leftIntakeMotor;
    private final DcMotor rightIntakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntakeMotor = hardwareMap.get(DcMotor.class, "rightIntake");

        rightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeState = IntakeState.DUAL_IDLE;
    }

    private double power = 1.0;
    public void runTeleOp(Gamepad gamepad) {
        // Control Logic
        // RIGHT
        if (gamepad.right_trigger > 0) {
            rightIntakeMotor.setPower(1);
        } else {
            rightIntakeMotor.setPower(0);
        }
        // LEFT
        if (gamepad.left_trigger > 0) {
            leftIntakeMotor.setPower(1);
        } else {
            leftIntakeMotor.setPower(0);
        }
        // Adjust Power
        if (gamepad.rightBumperWasPressed()) {
            power += 0.05;
        } else if (gamepad.leftBumperWasPressed()) {
            power -= 0.05;
        }
    }

    public void spinLeftIntake() {
        leftIntakeMotor.setPower(1.0);
    }


    public double getIntakePower() {
        return power;
    }
}
