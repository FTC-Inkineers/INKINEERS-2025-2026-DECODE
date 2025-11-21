package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

@TeleOp(name="Shooter RPM Tester", group="Testing")
public class ShooterRPMTester extends OpMode {
    private ShooterSubsystem shooter;
    private VisionSubsystem vision;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, true);
        shooter.initTeleOp(vision);
    }

    @Override
    public void loop() {
        shooter.runRpmTester(gamepad1);

        telemetry.addData("Target RPM", shooter.getTestRPM());
        telemetry.addData("Increment by 100", "DPAD UP");
        telemetry.addData("Decrement by 100", "DPAD DOWN");
        telemetry.addData("Increment by 10", "Y");
        telemetry.addData("Decrement by 10", "X");

        shooter.sendAllTelemetry(telemetry, true);
        vision.sendTelemetry(telemetry, true);

        telemetry.update();
    }
}
