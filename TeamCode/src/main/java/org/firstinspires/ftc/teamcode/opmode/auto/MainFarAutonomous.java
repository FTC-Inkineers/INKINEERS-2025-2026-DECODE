package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.LEFT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeUnitState.INTAKE;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.auto.action.CannonSailClose;
import org.firstinspires.ftc.teamcode.opmode.auto.action.Navigator;
import org.firstinspires.ftc.teamcode.opmode.auto.action.CannonSailFar;
import org.firstinspires.ftc.teamcode.pedroPathing.FarPaths;
import org.firstinspires.ftc.teamcode.subsystem.RGBSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public abstract class MainFarAutonomous extends OpMode {
    protected DriveSubsystem drive;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected VisionSubsystem vision;
    protected RGBSubsystem rgb;
    protected FarPaths paths;
    protected Navigator navigator;

    protected enum Variant {
        SOLO,
        CARRY,
        SYNERGIZED
    }

    private VisionSubsystem.ObeliskMotif motif = VisionSubsystem.ObeliskMotif.UNKNOWN;

    // Abstract methods to be implemented
    protected abstract boolean isBlueSide();
    protected abstract Variant getVariant();

    // Timer variables

    private ElapsedTime pathTimer, opmodeTimer;
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
        paths = new FarPaths(drive.follower, isBlueSide());
        navigator = new Navigator();

        // Make sure to call this AFTER paths have been constructed.
        drive.initAuto(paths.START_POSE);

        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
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
    public void start() {
        pathTimer.reset();
        opmodeTimer.reset();
        setPathState(0);

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
        drive.sendAllTelemetry(telemetry, true);
        shooter.sendAllTelemetry(telemetry, false);
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        Variant variant = getVariant(); // Set the variant here

        switch(pathState) {
            case 0:
                // First cycle (Far Shot)
                if (runCycle(paths.Path1, paths.Path2, true)) {
                    setPathState(1);
                }
                break;

            case 1:
                // Second cycle (Far Shot)
                if (runCycle(paths.Path3, paths.Path4, true)) {
                    setPathState(2);
                }
                break;

            case 2:
                // Third cycle logic depends on the selected variant
                if (variant == Variant.SOLO) {
                    // Third cycle (Close Shot)
                    if (runCycle(paths.Path5, paths.Path6, false)) {
                        setPathState(3); // Move to park after 3rd cycle
                    }
                } else {
                    // For any other variant (like TWO_CYCLES_AND_PARK), skip straight to parking
                    setPathState(3);
                }
                break;

            case 3:
                // Add parking logic here
                // For example:
                // drive.follower.followPath(paths.PARK_CENTER);
                // setPathState(4);
                break;

            case 4:
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

    private boolean runCycle(PathChain toShootPath, PathChain toIntakePath, boolean isFarShot) {
        switch (cycleState) {
            case IDLE:
                // Start moving to the shooting position and spin up the shooter
                drive.follower.followPath(toShootPath, 0.6, true);
                shooter.setTargetRPM(isFarShot ? shooter.getStationaryRPM_Far() : shooter.getStationaryRPM_Close());
                cycleState = CycleState.MOVING_TO_SHOOT;
                break;

            case MOVING_TO_SHOOT:
                // Continue spinning up the shooter while moving
                shooter.updateShooterPower();
                // When the robot arrives, shoot
                if (!drive.follower.isBusy()) {
                    autoShoot(isFarShot);
                    cycleState = CycleState.SHOOTING;
                }
                break;

            case SHOOTING:
                // Wait for the shooting sequence to complete
                if (shooter.isIdle()) {
                    // Start intaking and move to the next intake position
                    autoIntake();
                    drive.follower.followPath(toIntakePath, 0.6, true);
                    cycleState = CycleState.MOVING_TO_INTAKE;
                }
                break;

            case MOVING_TO_INTAKE:
                // Wait for the robot to arrive at the intake spot
                if (!drive.follower.isBusy()) {
                    intake.stop();
                    cycleState = CycleState.IDLE; // Reset for the next cycle
                    return true; // Signal that this cycle is complete
                }
                break;
        }
        return false; // Cycle is still in progress
    }

    public void autoShoot(boolean farShot) {
        navigator.setSail(farShot ? new CannonSailFar(shooter, intake, motif) : new CannonSailClose(shooter, intake, motif));
    }

    public void autoIntake() {
        if (isBlueSide()) {
            intake.setIntake(LEFT, INTAKE);
        } else {
            intake.setIntake(RIGHT, INTAKE);
        }
    }
}
