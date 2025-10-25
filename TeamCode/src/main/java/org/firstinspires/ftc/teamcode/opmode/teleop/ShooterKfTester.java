package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@TeleOp(name = "Shooter kF Tuner", group = "Testing")
public class ShooterKfTester extends OpMode {

    // Subsystems
    private ShooterSubsystem shooterSubsystem;

    @Override
    public void init() {
        // Subsystem Init
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
    }

    @Override
    public void loop() {

        shooterSubsystem.runKfTester(gamepad1);

        // Telemetry
        shooterSubsystem.enableAllTelemetry(this, true);
        telemetry.addData("Shooter Power", shooterSubsystem.testPower);
        telemetry.addData("Change Factor", shooterSubsystem.changeFactor);
        telemetry.update();
    }
}
