package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.CannonSailFar;
import org.firstinspires.ftc.teamcode.utility.Navigator;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.RGBSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utility.SequenceMapper;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name = "Sail Testing", group = "Testing")
public class SailTester extends OpMode {
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private VisionSubsystem vision;
    private RGBSubsystem rgb;
    private Navigator navigator;
    private VisionSubsystem.ObeliskMotif motif = VisionSubsystem.ObeliskMotif.UNKNOWN;
    private SequenceMapper sequenceMapper;



    @Override
    public void init() {
        vision = new VisionSubsystem(hardwareMap, true);
        rgb = new RGBSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        navigator = new Navigator();
        sequenceMapper = new SequenceMapper();
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
        rgb.runAuto(shooter.getCurrentRPM());
        navigator.update();


        if (gamepad1.aWasPressed()) {
            navigator.setSail(new CannonSailFar(shooter, intake, getShootingSequence(motif), 1));
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

    protected SequenceMapper.Sequence getShootingSequence(VisionSubsystem.ObeliskMotif motif) {
        SequenceMapper.PositionConfig target;
        switch (motif) {
            case GPP: target = SequenceMapper.PositionConfig.GPP; break;
            case PPG: target = SequenceMapper.PositionConfig.PPG; break;
            case PGP:
            default: target = SequenceMapper.PositionConfig.PGP; break;
        }

        // Assuming current config is GPP (Green Left)
        SequenceMapper.PositionConfig current = SequenceMapper.PositionConfig.GPP;

        return sequenceMapper.getMappedSequenceEnum(target, current);
    }
}
