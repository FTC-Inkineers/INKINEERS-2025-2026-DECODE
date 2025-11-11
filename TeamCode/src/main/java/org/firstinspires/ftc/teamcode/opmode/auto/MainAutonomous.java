package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.auto.action.ActionRunner;
import org.firstinspires.ftc.teamcode.opmode.auto.action.ShootAction;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public abstract class MainAutonomous extends OpMode {
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Paths paths;
    ActionRunner actionRunner;

    protected abstract boolean isBlueSide();

    private ElapsedTime pathTimer, opmodeTimer;
    private int pathState;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, isBlueSide());
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        paths = new Paths(drive.follower, isBlueSide());
        actionRunner = new ActionRunner();

        drive.initAuto();

        pathTimer = new ElapsedTime();;
        opmodeTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        pathTimer.reset();
        opmodeTimer.reset();
        setPathState(0);
    }

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        drive.follower.update();
        shooter.runAuto();
        actionRunner.update();

        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("Opmode Time", opmodeTimer.toString());
        telemetry.addData("Path Timer", pathTimer.toString());
        drive.sendAllTelemetry(telemetry, true);
        shooter.sendAllTelemetry(telemetry, false);
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // State 0: Start following Path 1
                drive.follower.followPath(paths.Path1, true);
                setPathState(1);
                break;

            case 1:
                // State 1: Wait for Path 1 to finish, then shoot
                if (!drive.follower.isBusy()) {
                    actionRunner.runAction(new ShootAction(shooter, intake));
                    setPathState(2);
                }
                break;

            case 2:
                // State 2: Wait for shooting to complete, then start Path 2 with intake
                if (shooter.isIdle()) { // Wait for shooter
                    intake.setRightIntake(1); // Turn on intake
                    drive.follower.followPath(paths.Path2, 0.6, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!drive.follower.isBusy()) {
                    intake.stop();
                    drive.follower.followPath(paths.Path3, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!drive.follower.isBusy()) {
//                    shooter.autoShoot();
                    setPathState(5);
                }
                break;

            case 5:
                if (shooter.isIdle()) {
                    intake.setRightIntake(1);
                    drive.follower.followPath(paths.Path4, 0.6, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!drive.follower.isBusy()) {
                    intake.stop();
                    drive.follower.followPath(paths.Path5, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!drive.follower.isBusy()) {
                    shooter.autoShoot(intake);
                    setPathState(8);
                }
                break;

            case 8:
                if (shooter.isIdle()) {
                    intake.setRightIntake(1);
                    drive.follower.followPath(paths.Path6, 0.6, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!drive.follower.isBusy()) {
                    intake.stop();
                    drive.follower.followPath(paths.Path7, true);
                    setPathState(10); // Move to idle state
                }
                break;

            case 10:
                if (!drive.follower.isBusy()) {
                    shooter.autoShoot(intake);
                    setPathState(11);
                }

                break;

            case 11:

                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    public void shoot() {
        shooter.autoShoot(intake);
    }
}
