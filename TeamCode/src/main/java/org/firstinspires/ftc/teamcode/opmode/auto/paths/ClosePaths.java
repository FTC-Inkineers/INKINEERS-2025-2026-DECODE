package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import static org.firstinspires.ftc.teamcode.RobotConstants.DEFAULT_BRAKING_STRENGTH;
import static org.firstinspires.ftc.teamcode.RobotConstants.DEFAULT_VELOCITY_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.RobotConstants.PARKING_VELOCITY_CONSTRAINT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.opmode.auto.MainCloseAutonomous;

/** @noinspection FieldCanBeLocal*/
public class ClosePaths {
    // region MASTER PATH DEFINITIONS (Default to BLUE alliance)
    // These variables will be used as-is for BLUE or reflected for RED.

    // START to Scan Pose
    private Pose p1_c1 = new Pose(30.000, 8.000);
    private Pose p1_start = new Pose(24.000, 132.000);
    private Pose p1_end   = new Pose(48.000, 108.000);
    private double p1_start_h = Math.toRadians(225);
    private double p1_end_h   = Math.toRadians(170);

    public Pose START_POSE = new Pose(p1_start.getX(), p1_start.getY(), p1_start_h);

    // Scan Pose to Shoot Pose 1
    private Pose p2_end = new Pose(60.000, 84.000);
    private double p2_end_h = Math.toRadians(220);

    // Shoot pose 3 to Intake 3
    private Pose p6_end = new Pose(12.000, 84.000);
    private double p6_end_h = Math.toRadians(180);

    private Pose pClose_end = new Pose(55.000, 60.000);
    private double pClose_end_h = Math.toRadians(180);
    // endregion


    // PathChain member variables, to be initialized in the constructor
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

    /**
     * The main constructor that builds paths for the specified alliance.
     * @param follower The Follower object from your drive train.
     */
    public ClosePaths(Follower follower, boolean isBlueSide, MainCloseAutonomous.Species species) {
        // If RED alliance, reflect all master poses and headings.
        // If BLUE, this block is skipped and the default values are used.
        if (!isBlueSide) {
            p1_c1 = reflect(p1_c1);
            p1_start = reflect(p1_start);
            p1_end = reflect(p1_end);
            p1_start_h = reflect(p1_start_h);
            p1_end_h = reflect(p1_end_h);

            p2_end = reflect(p2_end);
            p2_end_h = reflect(p2_end_h);



            p6_end = reflect(p6_end);
            p6_end_h = reflect(p6_end_h);

            pClose_end = reflect(pClose_end);
            pClose_end_h = reflect(pClose_end_h);

            START_POSE = new Pose(p1_start.getX(), p1_start.getY(), p1_start_h);
        }

        // Start to Scan Pose
        Path1 = follower.pathBuilder()
                .addPath(new BezierCurve(p1_start, p1_c1, p1_end))
                .setLinearHeadingInterpolation(p1_start_h, p1_end_h)
                .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                .build();

        // Scan Pose to Shoot Pose 1
        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(p1_end, p2_end))
                .setLinearHeadingInterpolation(p1_end_h, p2_end_h)
                .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                .build();

        // Shoot Pose 1 to PARK
        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(p2_end, pClose_end))
                .setLinearHeadingInterpolation(p2_end_h, pClose_end_h)
                .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                .setVelocityConstraint(PARKING_VELOCITY_CONSTRAINT)
                .build();
    }

    /**
     * Helper method to reflect a Pose over the vertical line x = 72.
     * @param pose The original Pose.
     * @return The reflected Pose.
     */
    private Pose reflect(Pose pose) {
        // We pass 0 for heading here because the heading is reflected separately.
        return new Pose(144 - pose.getX(), pose.getY(), 0);
    }

    /**
     * Helper method to reflect a heading angle over a vertical line.
     * The new angle is (PI - old angle).
     * @param angleInRadians The original angle in radians.
     * @return The reflected angle in radians.
     */
    private double reflect(double angleInRadians) {
        return Math.PI - angleInRadians;
    }
}
