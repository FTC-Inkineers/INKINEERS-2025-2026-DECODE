package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@Config
public abstract class AquamarineDrive extends OpMode {
    protected DriveSubsystem drive;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected FtcDashboard dashboard;
    protected PanelsTelemetry panels;

    private ElapsedTime loopTimer;

    protected abstract boolean isBlueSide();

    // Telemetry Choices
    public static boolean allDriveTelemetry = true;
    public static boolean allShooterTelemetry = false;
    public static boolean allIntakeTelemetry = false;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, isBlueSide());
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, isBlueSide());
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        panels = PanelsTelemetry.INSTANCE;


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
        shooter.runTeleOp(gamepad2);
        intake.runTeleOp(gamepad1);

        // Telemetry
        drive.enableAllTelemetry(this, allDriveTelemetry);
        shooter.enableAllTelemetry(this, allShooterTelemetry);
        intake.enableAllTelemetry(this, allIntakeTelemetry);
        panels.getTelemetry().addData("Target RPM", shooter.getTargetRPM());
        panels.getTelemetry().addData("Shooter RPM", shooter.getCurrentRPM());

        telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
        telemetry.update();

        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            // Restore the interrupted status
            Thread.currentThread().interrupt();
        }
    }
}
