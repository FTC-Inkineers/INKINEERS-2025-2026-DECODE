package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opmode.auto.action.FireCannon;
import org.firstinspires.ftc.teamcode.opmode.auto.action.NavigationConsole;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public class ActionTester extends OpMode {
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    NavigationConsole navigationConsole;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, true);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        navigationConsole = new NavigationConsole();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            navigationConsole.setSail(new FireCannon(shooter, intake));
        }
    }
}
