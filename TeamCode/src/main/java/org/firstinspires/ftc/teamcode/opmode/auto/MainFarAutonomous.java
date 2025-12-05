package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.LEFT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeUnitState.INTAKE;

import androidx.annotation.Nullable;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utility.CannonSailClose;
import org.firstinspires.ftc.teamcode.utility.Navigator;
import org.firstinspires.ftc.teamcode.utility.CannonSailFar;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.FarPaths;
import org.firstinspires.ftc.teamcode.subsystem.RGBSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.utility.SequenceMapper;

public abstract class MainFarAutonomous extends OpMode {
    protected DriveSubsystem drive;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected VisionSubsystem vision;
    protected RGBSubsystem rgb;
    protected FarPaths paths;
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
        paths = new FarPaths(drive.follower, isBlueSide(), getVariant());
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
        Species variant = getVariant(); // Set the variant here

        switch(pathState) {
            case -1:
                // Wait for our alliance if needed
                if (opmodeTimer.seconds() > delayTime)
                    setPathState(0);
                break;
            case 0:
                // First cycle (Far Shot) - Index 0 (Preload)
                if (runCycle(paths.Path1, paths.Path2, true, 0)) {
                    setPathState(1);
                }
                break;

            case 1:
                // Second cycle (Far Shot) - Index 3
                if (runCycle(paths.Path3, paths.Path4, true, 3)) {
                    setPathState(2);
                }
                break;

            case 2:
                // Third cycle (Far OR Close Shot) - Index 2
                if (variant == Species.SOLO) {
                    // Third cycle (Close Shot)
                    if (runCycle(paths.Path5, paths.Path6, false, 2)) {
                        setPathState(3); // Move to park after 3rd cycle
                    }
                } else {
                    if (runCycle(paths.Path5, null, true, 2)) {
                        setPathState(4);
                    }
                }
                break;

            case 3:
                // Fourth cycle logic depends on the selected variant
                // Assumed to be index 2 again or another index if specified?
                // For now, keeping it consistent with the previous logic or user request.
                // User said "only shoot index 0, index 3, and optionally 2".
                // Cycle 4 for SOLO uses Path7.
                if (variant == Species.SOLO) {
                    // Third cycle (Close Shot) - Repeating Index 2 or generic?
                    // Assuming Index 2 based on "optionally 2".
                    if (runCycle(paths.Path7, null, false, 2)) {
                        setPathState(4); // Move to park after 3rd cycle
                    }
                } else {
                    setPathState(4);
                }
                break;

            case 4:
                park();
                setPathState(5);
                break;

            case 5:
                // Wait for parking to finish
                if (!drive.follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6:
                // Opmode finished
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
        cycleState = CycleState.IDLE; // Reset cycle state machine for the new path state
    }

    private boolean runCycle(PathChain toShootPath, @Nullable PathChain toIntakePath, boolean isFarShot, int shotIndex) {
        switch (cycleState) {
            case IDLE:
                // Start moving to the shooting position and spin up the shooter
                drive.follower.followPath(toShootPath, 0.8, true);
                shooter.setTargetRPM(isFarShot ? shooter.getStationaryRPM_Far() : shooter.getStationaryRPM_Close());
                // Update the telemetry index for debugging
                this.index = shotIndex;
                cycleState = CycleState.MOVING_TO_SHOOT;
                break;

            case MOVING_TO_SHOOT:
                // Continue spinning up the shooter while moving
                shooter.updateShooterPhysics();
                // Stop intake 0.5 seconds later to ensure elements
                if (actionTimer.seconds() > 0.5)
                    intake.stop();
                // When the robot arrives, shoot
                if (!drive.follower.isBusy()) {
                    autoShoot(shotIndex, isFarShot);
                    cycleState = CycleState.SHOOTING;
                }
                break;

            case SHOOTING:
                // Wait for the shooting sequence to complete
                if (shooter.isIdle()) {
                    if (toIntakePath != null) {
                        // Start intaking and move to the next intake position
                        autoIntake();
                        drive.follower.followPath(toIntakePath, 0.8, true);
                        cycleState = CycleState.MOVING_TO_INTAKE;
                    } else {
                        cycleState = CycleState.IDLE;
                        return true; // Signal that this cycle is complete
                    }
                }
                break;

            case MOVING_TO_INTAKE:
                // When the robot finishes the intake path...
                if (!drive.follower.isBusy()) {
                    actionTimer.reset();
                    cycleState = CycleState.IDLE; // Reset for the next cycle
                    return true; // Signal that this cycle is complete
                }
                break;
        }
        return false; // Cycle is still in progress
    }

    private void park() {
        Species variant = getVariant();
        PathChain parkPath;
        intake.stop();

        switch (variant) {
            case SOLO:
                // The SOLO variant parks after the 3rd cycle, using Path8
                parkPath = paths.Path8;
                break;
            case PUSH:
            case SYMBIOTIC:
                // PUSH and SYMBIOTIC variants park after the 2nd cycle, using Path6
                parkPath = paths.Path6;
                break;
            default:
                // Default to a safe parking path if something goes wrong.
                // You might want to define a default park path in FarPaths.
                parkPath = paths.Path6; // Assuming Path6 is a safe default
                break;
        }

        drive.follower.followPath(parkPath, 0.8, true);
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
