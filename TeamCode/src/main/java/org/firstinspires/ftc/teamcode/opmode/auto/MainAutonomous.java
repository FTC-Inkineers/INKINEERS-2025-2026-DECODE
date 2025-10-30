package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public abstract class MainAutonomous extends OpMode {
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Paths paths;

    protected abstract boolean isBlueSide();

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, isBlueSide());
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, isBlueSide());
        paths = new Paths(drive.follower, isBlueSide());

        drive.initAuto();
    }

    @Override
    public void loop() {

    }
}
