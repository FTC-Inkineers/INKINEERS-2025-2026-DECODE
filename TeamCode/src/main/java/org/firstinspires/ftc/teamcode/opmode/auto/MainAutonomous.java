package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.LEFT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeUnitState.INTAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.auto.action.Navigator;
import org.firstinspires.ftc.teamcode.opmode.auto.action.CannonSail;
import org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public abstract class MainAutonomous extends OpMode {
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Paths paths;
    Navigator navigator;

    protected abstract boolean isBlueSide();

    private ElapsedTime pathTimer, opmodeTimer;
    private int pathState;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, isBlueSide());
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        paths = new Paths(drive.follower, isBlueSide());
        navigator = new Navigator();

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
        intake.runAuto();
        navigator.update();

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
                // Start following Path and Spin Up early
                drive.follower.followPath(paths.Path1, true);
                shooter.setTargetRPM(shooter.getStationaryRPM());
                setPathState(1);
                break;

            case 1:
                // Spin up while waiting for path to finish, then shoot
                shooter.updateShooterPower();

                if (!drive.follower.isBusy()) {
                    autoShoot();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for shooter to finish before following next Path
                if (shooter.isIdle()) {
                    autoIntake();
                    drive.follower.followPath(paths.Path2, 0.6, true);
                    setPathState(3);
                }
                break;

            case 3:
                // Start following Path and Spin Up early
                if (!drive.follower.isBusy()) {
                    intake.stop();
                    drive.follower.followPath(paths.Path3, true);
                    shooter.setTargetRPM(shooter.getStationaryRPM());
                    setPathState(4);
                }
                break;

            case 4:
                // Spin up while waiting for path to finish, then shoot
                shooter.updateShooterPower();

                if (!drive.follower.isBusy()) {
                    autoShoot();
                    setPathState(5);
                }
                break;

            case 5:
                // Wait for shooter to finish before following next Path
                if (shooter.isIdle()) {
                    autoIntake();
                    drive.follower.followPath(paths.Path4, 0.6, true);
                    setPathState(6);
                }
                break;

            case 6:
                // Start following Path and Spin Up early
                if (!drive.follower.isBusy()) {
                    intake.stop();
                    drive.follower.followPath(paths.Path5, true);
                    shooter.setTargetRPM(shooter.getStationaryRPM());
                    setPathState(7);
                }
                break;

            case 7:
                // Spin up while waiting for path to finish, then shoot
                shooter.updateShooterPower();

                if (!drive.follower.isBusy()) {
                    autoShoot();
                    setPathState(8);
                }
                break;

            case 8:
                // Wait for shooter to finish before following next Path
                if (shooter.isIdle()) {
                    autoIntake();
                    drive.follower.followPath(paths.Path6, 0.6, true);
                    setPathState(9);
                }
                break;

            case 9:
                // Start following Path and Spin Up early
                if (!drive.follower.isBusy()) {
                    intake.stop();
                    drive.follower.followPath(paths.Path7, true);
                    shooter.setTargetRPM(shooter.getStationaryRPM());
                    setPathState(10);
                }
                break;

            case 10:
                // Spin up while waiting for path to finish, then shoot
                shooter.updateShooterPower();

                if (!drive.follower.isBusy()) {
                    autoShoot();
                    setPathState(11);
                }
                break;

            case 11:
                // TODO: Move out of shooting zone.
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    public void autoShoot() {
        navigator.setSail(new CannonSail(shooter, intake));
    }

    public void autoIntake() {
        if (isBlueSide()) {
            intake.setIntake(LEFT, INTAKE);
        } else {
            intake.setIntake(RIGHT, INTAKE);
        }
    }
}
