package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@TeleOp(name = "Shooter Tester", group = "Testing")
public class ShooterTester extends OpMode {

    PanelsTelemetry panels;

    private ShooterSubsystem shooterSubsystem;

    @Override
    public void init() {
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        panels = PanelsTelemetry.INSTANCE;
    }

    @Override
    public void loop() {
        shooterSubsystem.runTeleOp(gamepad1);

        // Telemetry
        shooterSubsystem.enableAllTelemetry(this, true);
        telemetry.update();

        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            // Restore the interrupted status
            Thread.currentThread().interrupt();
        }
    }
}
