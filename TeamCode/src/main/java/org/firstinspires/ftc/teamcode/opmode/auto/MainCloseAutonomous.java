package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.LEFT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeUnitState.INTAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.auto.action.CannonSailClose;
import org.firstinspires.ftc.teamcode.opmode.auto.action.CannonSailFar;
import org.firstinspires.ftc.teamcode.opmode.auto.action.Navigator;
import org.firstinspires.ftc.teamcode.opmode.auto.paths.ClosePaths;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.RGBSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

public abstract class MainCloseAutonomous extends OpMode {
    protected DriveSubsystem drive;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected VisionSubsystem vision;
    protected RGBSubsystem rgb;
    protected ClosePaths paths;
    protected Navigator navigator;

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
                    // Shoot once arrived
                    navigator.setSail(new CannonSailClose(shooter, intake, motif, 1));
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
        navigator.setSail(farShot ? new CannonSailFar(shooter, intake, motif, index) : new CannonSailClose(shooter, intake, motif, index));
    }

    public void autoIntake() {
        if (isBlueSide()) {
            intake.setIntake(LEFT, INTAKE);
        } else {
            intake.setIntake(RIGHT, INTAKE);
        }
    }
}
