package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.auto.action.CannonSail;
import org.firstinspires.ftc.teamcode.opmode.auto.action.Navigator;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@TeleOp(name = "Sail Testing", group = "Testing")
public class SailTester extends OpMode {
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Navigator navigator;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, true);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        navigator = new Navigator();
    }

    @Override
    public void loop() {
        shooter.runAuto();
        intake.runAuto();
        navigator.update();


        if (gamepad1.aWasPressed()) {
            navigator.setSail(new CannonSail(shooter, intake));
        }

        telemetry.addData("Shooting Sequence", "Button A");
        telemetry.update();
    }
}
