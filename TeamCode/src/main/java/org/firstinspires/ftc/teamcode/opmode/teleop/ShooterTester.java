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
        // Gamepad Init
        loopTimer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
    }

    public void start() {
    }

    @Override
    public void loop() {
        // Track Loop Time
        loopTimer.reset();

        // Control Logic
        if (gamepad1.rightBumperWasPressed()) {
            shooterSubsystem.fire();
        }
        if (gamepad1.right_trigger > 0) {
            shooterSubsystem.spinUp();
        } else if (shooterSubsystem.getShooterPower() >= 1.0) {
            shooterSubsystem.spinRelease();
        }


        shooterSubsystem.update();

        // Telemetry
        telemetry.addData("Shooter Message", shooterSubsystem.shooterMessage());
        telemetry.addData("Trigger Time", shooterSubsystem.getTimerSeconds());

        telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
        telemetry.update();
    }
}
