package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public abstract class MainAutonomous extends OpMode {
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    Paths paths;

    protected abstract boolean isBlueSide();

    private ElapsedTime pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, isBlueSide());
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap, isBlueSide());
        paths = new Paths(drive.follower, isBlueSide());

        drive.initAuto();
    }

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        drive.follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        drive.enableAllTelemetry(this, true);
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
