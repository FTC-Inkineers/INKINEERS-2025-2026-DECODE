// File: org/firstinspires/ftc/teamcode/subsystem/AquamarineRobot.java
package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AquamarineRobot {

    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    public static boolean enableALlDriveTelemetry = false;
    public static boolean enableAllShooterTelemetry = false;
    public static boolean enableAllIntakeTelemetry = false;

    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public AquamarineRobot(HardwareMap hardwareMap, boolean isBlueSide) {
        drive = new DriveSubsystem(hardwareMap, isBlueSide);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
    }

    public void initTeleOp(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        drive.initTeleOp(gamepad1, gamepad2);
    }

    public void start() {
        drive.start();
    }

    public void runTeleOp() {
        drive.runTeleOp(shooter.isActive());
        shooter.runTeleOp(gamepad2);
        intake.runTeleOp(gamepad1);
    }

    public void sendTelemetry(Telemetry telemetry) {
        shooter.sendAllTelemetry(telemetry, enableAllShooterTelemetry);
        drive.sendAllTelemetry(telemetry, enableALlDriveTelemetry);
        intake.sendAllTelemetry(telemetry, enableAllIntakeTelemetry);
        drive.sendAprilTagTelemetry(telemetry);
        // Add other relevant telemetry from other subsystems if needed
    }
}
