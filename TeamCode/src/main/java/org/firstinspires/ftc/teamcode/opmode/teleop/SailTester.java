package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utility.CannonSailClose;
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

    private SequenceMapper.Sequence shootingSequence = SequenceMapper.Sequence.LMR;

    @Override
    public void init() {
        vision = new VisionSubsystem(hardwareMap, false);
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

    private int configIndex = 0;
    @Override
    public void loop() {
        shooter.runAuto();
        intake.runAuto();
        rgb.runAuto(shooter.getCurrentRPM());
        navigator.update();

        if (gamepad1.aWasPressed()) {
            autoShoot(configIndex, true);
        } else if (gamepad1.dpadUpWasPressed()) {
            configIndex += 1;
            if (configIndex > 4) {
                configIndex = 0;
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            configIndex -= 1;
            if (configIndex < 0) {
                configIndex = 4;
            }
        }

        if (gamepad1.y) {
            vision.update();
            motif = vision.detectObeliskMotif();
        }

        telemetry.addData("Pattern Locked", motif);
        telemetry.addData("Pattern Index", configIndex);
        telemetry.addData("Pattern Config", getConfigForIndex(configIndex));
        telemetry.addData("Shooting Sequence", shootingSequence);
        telemetry.addData("Redetect Tag", "Button Y");
        telemetry.addData("Shooting Sequence", "Button A");
        telemetry.addData("TargetRPM", shooter.getTargetRPM());
        telemetry.update();
    }

    public void autoShoot(int index, boolean farShot) {
        SequenceMapper.Sequence sequence = getShootingSequence(motif, index);
        shootingSequence = sequence;
        navigator.setSail(farShot ? new CannonSailFar(shooter, intake, sequence) : new CannonSailClose(shooter, intake, sequence));
    }

    protected SequenceMapper.Sequence getShootingSequence(VisionSubsystem.ObeliskMotif motif, int index) {
        // 1. Determine what the pattern looks like on the wall (Target)
        SequenceMapper.PositionConfig target = getTargetConfigFromMotif(motif);

        // 2. Determine what the pattern looks like in the robot's intake (Source)
        SequenceMapper.PositionConfig current = getConfigForIndex(index);

        // 3. Calculate the sequence required to transform Source -> Target
        return sequenceMapper.getMappedSequenceEnum(target, current);
    }

    protected SequenceMapper.PositionConfig getConfigForIndex(int index) {
        SequenceMapper.PositionConfig blueConfig;

        switch (index) {
            case 0:
                blueConfig = SequenceMapper.PositionConfig.PPG; // Preload
                break;
            case 1:
                blueConfig = SequenceMapper.PositionConfig.GPP;
                break;
            case 2:
                blueConfig = SequenceMapper.PositionConfig.PGP;
                break;
            case 3:
                blueConfig = SequenceMapper.PositionConfig.PPG;
                break;
            case 4:
                blueConfig = SequenceMapper.PositionConfig.PGP;
                break;
            default:
                blueConfig = SequenceMapper.PositionConfig.GPP;
                break;
        }

        // RED SIDE TESTING
        return SequenceMapper.getMirroredConfig(blueConfig);
    }

    protected SequenceMapper.PositionConfig getTargetConfigFromMotif(VisionSubsystem.ObeliskMotif motif) {
        switch (motif) {
            case GPP: return SequenceMapper.PositionConfig.GPP;
            case PPG: return SequenceMapper.PositionConfig.PPG;
            case PGP: return SequenceMapper.PositionConfig.PGP;
            default:  return SequenceMapper.PositionConfig.PGP; // Default if vision fails
        }
    }
}
