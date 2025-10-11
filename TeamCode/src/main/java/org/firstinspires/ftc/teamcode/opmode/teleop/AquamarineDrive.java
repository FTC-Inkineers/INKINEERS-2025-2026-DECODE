package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@TeleOp(name = "Aquamarine Drive", group = "TeleOp")
public class AquamarineDrive extends OpMode {
    DriveSubsystem drive;
    ShooterSubsystem shooter;

    private ElapsedTime loopTimer;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap);
        drive.initTeleOp(gamepad1, gamepad2);
        shooter = new ShooterSubsystem(hardwareMap);

        loopTimer = new ElapsedTime();
    }
    @Override
    public void start() {

        drive.start();
    }
    @Override
    public void loop() {
        // Track Loop Time
        loopTimer.reset();

        drive.runTeleOp();
        shooter.runTeleOp(gamepad1);

        // Telemetry
        telemetry.addData("Shooter Message", shooter.shooterMessage());
        telemetry.addData("Trigger Time", shooter.getTimerSeconds());

        telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
        telemetry.update();
    }
}
