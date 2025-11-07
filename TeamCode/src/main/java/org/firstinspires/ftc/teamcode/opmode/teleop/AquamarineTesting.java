package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

@TeleOp(name = "SUBSYSTEM TESTING", group = "Testing")
public class AquamarineTesting extends AquamarineDrive {
    @Override
    protected boolean isBlueSide() {
        return true;
    }

    @Override
    protected boolean useDashboard() {
        return true;
    }

    @Override
    public void init() {
        boolean isBlue = true;

        // Testing Stuff
        boolean ALLIANCE_SELECTED = false;
        while (!ALLIANCE_SELECTED) {
            if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                ALLIANCE_SELECTED = true;
            } else if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                isBlue = false;
                ALLIANCE_SELECTED = true;
            }

            telemetry.addData("SELECT ALLIANCE", "X = BLUE | B = RED");
            telemetry.update();
        }

        drive = new DriveSubsystem(hardwareMap, isBlue);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        if (useDashboard()) {
            dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
        }


        drive.initTeleOp(gamepad1, gamepad2);
        loopTimer = new ElapsedTime();
    }
}
