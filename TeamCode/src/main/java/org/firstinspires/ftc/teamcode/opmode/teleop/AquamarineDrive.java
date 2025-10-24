package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public abstract class AquamarineDrive extends OpMode {
    protected DriveSubsystem drive;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;

    private ElapsedTime loopTimer;

    protected abstract boolean isBlueSide();

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, isBlueSide());
        intake = new IntakeSubsystem(hardwareMap, isBlueSide());
        shooter = new ShooterSubsystem(hardwareMap);

        drive.initTeleOp(gamepad1);

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
        intake.runTeleOp(gamepad1);

        // Telemetry
        shooter.enableAllTelemetry(this);
        intake.enableAllTelemetry(this);

        telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
        telemetry.update();
    }
}
