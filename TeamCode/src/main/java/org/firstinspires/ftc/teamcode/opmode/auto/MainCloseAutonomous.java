package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.LEFT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeUnitState.INTAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.CannonSailClose;
import org.firstinspires.ftc.teamcode.utility.CannonSailFar;
import org.firstinspires.ftc.teamcode.utility.Navigator;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.ClosePaths;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.RGBSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utility.SequenceMapper;

public abstract class MainCloseAutonomous extends OpMode {
    protected DriveSubsystem drive;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected VisionSubsystem vision;
    protected RGBSubsystem rgb;
    protected ClosePaths paths;
    protected Navigator navigator;
    protected SequenceMapper sequenceMapper;

    public enum Species {
        SOLO,
        PUSH,
        SYMBIOTIC
    }

    private VisionSubsystem.ObeliskMotif motif = VisionSubsystem.ObeliskMotif.UNKNOWN;

    // Abstract methods to be implemented
    protected abstract boolean isBlueSide();
    protected abstract Species getVariant();

    // Timer variables

    private ElapsedTime pathTimer, opmodeTimer, actionTimer;
    private double delayTime = 0.0;
    private int pathState;

    private enum CycleState {
        IDLE,
        MOVING_TO_SHOOT,
        SHOOTING,
        MOVING_TO_INTAKE
    }
    private CycleState cycleState = CycleState.IDLE;

    @Override
    public void init() {
        vision = new VisionSubsystem(hardwareMap, isBlueSide());
        rgb = new RGBSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap, vision, isBlueSide());
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        paths = new ClosePaths(drive.follower, isBlueSide(), getVariant());
        navigator = new Navigator();
        sequenceMapper = new SequenceMapper();

        // Make sure to call this AFTER paths have been constructed.
        drive.initAuto(paths.START_POSE);

        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        vision.update();
        motif = vision.detectObeliskMotif();

        if (gamepad1.rightBumperWasPressed())
            delayTime += 1.0;
        if (gamepad1.leftBumperWasPressed())
            delayTime -= 1.0;

        telemetry.addLine("Point the robot at the Obelisk to detect the pattern.");
        telemetry.addData("Detected Motif", motif);
        telemetry.addData("Delay Seconds", delayTime);
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.reset();
        opmodeTimer.reset();
        actionTimer.reset();
        setPathState(-1);

        telemetry.addData("Pattern Locked", motif);
        telemetry.update();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        drive.follower.update();
        shooter.runAuto();
        intake.runAuto();
        rgb.runAuto(shooter.getCurrentRPM());
        navigator.update();

        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("Opmode Time", opmodeTimer.toString());
        telemetry.addData("Path Timer", pathTimer.toString());
        telemetry.addData("Pattern Locked", motif);
        telemetry.addData("SHOOTING_INDEX", index);
        drive.sendAllTelemetry(telemetry, true);
        shooter.sendAllTelemetry(telemetry, false);
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case -1:
                // Optional delay for alliance coordination
                if (opmodeTimer.seconds() > delayTime) {
                    setPathState(0);
                }
                break;

            case 0:
                // Start moving to scan pose and spin up the shooter
                drive.follower.followPath(paths.Path1, 0.6, true);
                shooter.setTargetRPM(shooter.getStationaryRPM_Close());
                setPathState(1);
                break;

            case 1:
                // Set the motif.
                vision.update();
                motif = vision.detectObeliskMotif();
                // After scanning, start moving to shooting pose
                if (!drive.follower.isBusy()) {
                    drive.follower.followPath(paths.Path2, 0.6, true);
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for the robot to arrive at the shooting pose
                if (!drive.follower.isBusy()) {
                    // Shoot once arrived - Index 1
                    SequenceMapper.Sequence sequence = getShootingSequence(motif, 1);
                    navigator.setSail(new CannonSailClose(shooter, intake, sequence));
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for the shooting action to complete
                if (shooter.isIdle()) {
                    // Once shooting is done, move to park
                    drive.follower.followPath(paths.Path3, 0.6, true);
                    setPathState(4);
                }
                break;

            case 4:
                // Wait for parking to finish
                if (!drive.follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                // Opmode finished
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    private int index = 1;
    public void autoShoot(int index, boolean farShot) {
        SequenceMapper.Sequence sequence = getShootingSequence(motif, index);
        navigator.setSail(farShot ? new CannonSailFar(shooter, intake, sequence) : new CannonSailClose(shooter, intake, sequence));
    }

    public void autoIntake() {
        if (isBlueSide()) {
            intake.setIntake(LEFT, INTAKE);
        } else {
            intake.setIntake(RIGHT, INTAKE);
        }
    }

    protected SequenceMapper.Sequence getShootingSequence(VisionSubsystem.ObeliskMotif motif, int index) {
        // 1. Determine what the pattern looks like on the wall (Target)
        SequenceMapper.PositionConfig target = getTargetConfigFromMotif(motif);

        // 2. Determine what the pattern looks like in the robot's intake (Source)
        SequenceMapper.PositionConfig current = getConfigForIndex(index);

        // 3. Calculate the sequence required to transform Source -> Target
        return sequenceMapper.getMappedSequenceEnum(target, current);
    }

    /**
     * Maps a specific field index (0-4) to its physical ball configuration.
     *
     * Layout Definition (Blue Alliance Perspective):
     * Index 0 (Preload): GPP
     * Index 1: GPP
     * Index 2: PGP
     * Index 3: PPG
     * Index 4: PGP
     * (Totals: 2 GPP, 2 PGP, 1 PPG)
     */
    protected SequenceMapper.PositionConfig getConfigForIndex(int index) {
        SequenceMapper.PositionConfig blueConfig;

        switch (index) {
            case 0:
                blueConfig = SequenceMapper.PositionConfig.GPP; // Preload
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

        if (isBlueSide()) {
            return blueConfig;
        } else {
            return SequenceMapper.getMirroredConfig(blueConfig);
        }
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
