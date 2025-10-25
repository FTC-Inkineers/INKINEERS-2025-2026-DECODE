package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@TeleOp(name = "Shooter Tester", group = "Testing")
public class ShooterTester extends OpMode {

    // Subsystems
    private ShooterSubsystem shooterSubsystem;

    private ElapsedTime loopTimer;

    @Override
    public void init() {
        // Subsystem Init
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        loopTimer = new ElapsedTime();
    }

    @Override
    public void loop() {
        // Track Loop Time
        loopTimer.reset();

        shooterSubsystem.runTeleOp(gamepad1);

        // Telemetry
        shooterSubsystem.enableAllTelemetry(this, true);

        telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
        telemetry.update();
    }
}
