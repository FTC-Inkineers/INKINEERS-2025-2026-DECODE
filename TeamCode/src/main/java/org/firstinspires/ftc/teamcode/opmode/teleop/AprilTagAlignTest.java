package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

@TeleOp(name="April Tag Drive Test", group="Testing")
public class AprilTagAlignTest extends OpMode {

    private DriveSubsystem drive;
    private VisionSubsystem vision;


    @Override
    public void init() {
        vision = new VisionSubsystem(hardwareMap, true);
        drive = new DriveSubsystem(hardwareMap, vision, true);
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
        vision.sendTelemetry(telemetry, true);
        telemetry.update();
    }
}
