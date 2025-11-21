package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

@TeleOp(name = "Shooter Tester", group = "Testing")
public class ShooterTester extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    private ShooterSubsystem shooterSubsystem;

    @Override
    public void init() {
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        shooterSubsystem.initTeleOp(new VisionSubsystem(hardwareMap, true));
        telemetry = dashboard.getTelemetry();
    }

    @Override
    public void loop() {
        shooterSubsystem.runTeleOp(gamepad1);

        // Telemetry
        shooterSubsystem.sendAllTelemetry(telemetry, true);
        telemetry.update();

        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            // Restore the interrupted status
            Thread.currentThread().interrupt();
        }
    }
}
