package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

@TeleOp(name = "Intake Tester", group = "Testing")
public class IntakeTester extends OpMode {

    IntakeSubsystem intake;

    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap, true);
    }

    @Override
    public void loop() {
        intake.runTeleOp(gamepad1);
        telemetry.addData("Intake Power", intake.getIntakePower());
        telemetry.update();
    }
}
