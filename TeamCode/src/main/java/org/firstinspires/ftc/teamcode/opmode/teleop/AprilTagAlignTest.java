package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

@TeleOp(name="April Tag Drive Test", group="Testing")
public class AprilTagAlignTest extends OpMode {

    private DriveSubsystem drive;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, true);
        drive.initTeleOp(gamepad1, gamepad2);
    }

    @Override
    public void start() {
        drive.start();
    }

    @Override
    public void loop() {
        drive.runTeleOp();
        drive.sendAllTelemetry(telemetry, false);
        drive.sendAprilTagTelemetry(telemetry);
        telemetry.update();
    }
}
