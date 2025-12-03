package org.firstinspires.ftc.teamcode.utility;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

@Autonomous(name = "Test Path")
public class TestPath extends OpMode {
    DriveSubsystem drive;
    VisionSubsystem vision;

    private VisionSubsystem.ObeliskMotif motif = VisionSubsystem.ObeliskMotif.UNKNOWN;

    boolean isBlueSide = true;

    private ElapsedTime pathTimer, opmodeTimer;
    private int pathState;

    @Override
    public void init() {
        vision = new VisionSubsystem(hardwareMap, isBlueSide);
        drive = new DriveSubsystem(hardwareMap, vision, isBlueSide);

        drive.initAuto(new Pose());

        buildPaths();

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

        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("Pattern Locked", motif);
        drive.sendAllTelemetry(telemetry, true);
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // Start following Path and Spin Up early
                drive.follower.followPath(scorePreload, true);
                setPathState(1);
                break;

            case 1:
                // Spin up while waiting for path to finish, then shoot

                if (!drive.follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                telemetry.addLine("Done");
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }


    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(60, 0, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private Path scorePreload;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
    }
}
