package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

@TeleOp(name = "Intake Tester", group = "Testing")
public class IntakeTester extends OpMode {

    IntakeSubsystem intake;

    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        intake.runTeleOp(gamepad1);
        intake.enableAllTelemetry(this, true);
        telemetry.update();
    }
}
