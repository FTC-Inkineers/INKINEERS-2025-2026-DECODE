package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.auto.action.CannonSail;
import org.firstinspires.ftc.teamcode.opmode.auto.action.Navigator;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name = "Sail Testing", group = "Testing")
public class SailTester extends OpMode {
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private VisionSubsystem vision;
    private Navigator navigator;

    private VisionSubsystem.ObeliskMotif motif = VisionSubsystem.ObeliskMotif.UNKNOWN;


    @Override
    public void init() {
        vision = new VisionSubsystem(hardwareMap, true);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        navigator = new Navigator();
    }

    @Override
    public void init_loop() {
        vision.update();
        motif = vision.detectObeliskMotif();

        telemetry.addLine("Point the robot at the Obelisk to detect the pattern.");
        telemetry.addData("Detected Motif", motif);
        telemetry.update();
    }

    @Override
    public void loop() {
        shooter.runAuto();
        intake.runAuto();
        navigator.update();


        if (gamepad1.aWasPressed()) {
            navigator.setSail(new CannonSail(shooter, intake, motif));
        }

        if (gamepad1.y) {
            vision.update();
            motif = vision.detectObeliskMotif();
        }

        telemetry.addData("Pattern Locked", motif);
        telemetry.addData("Redetect Tag", "Button Y");
        telemetry.addData("Shooting Sequence", "Button A");
        telemetry.addData("TargetRPM", shooter.getTargetRPM());
        telemetry.update();
    }
}
