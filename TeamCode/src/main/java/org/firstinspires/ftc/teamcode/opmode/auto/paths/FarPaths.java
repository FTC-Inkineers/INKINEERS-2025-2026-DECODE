package org.firstinspires.ftc.teamcode.opmode.auto.paths;

import static org.firstinspires.ftc.teamcode.RobotConstants.DEFAULT_BRAKING_STRENGTH;
import static org.firstinspires.ftc.teamcode.RobotConstants.DEFAULT_VELOCITY_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.RobotConstants.PARKING_VELOCITY_CONSTRAINT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.opmode.auto.MainFarAutonomous;

/** @noinspection FieldCanBeLocal*/
public class FarPaths {
    // region MASTER PATH DEFINITIONS (Default to BLUE alliance)
    // These variables will be used as-is for BLUE or reflected for RED.

    // START to Shoot pose 1
    private Pose p1_c1 = new Pose(30.000, 8.000);
    private Pose p1_start = new Pose(64.000, 8.000);
    private Pose p1_end   = new Pose(60.000, 17.000);
    private double p1_start_h = Math.toRadians(180);
    private double p1_end_h   = Math.toRadians(200);

    public Pose START_POSE = new Pose(p1_start.getX(), p1_start.getY(), p1_start_h);

    // Shoot pose 1 to Intake 1
    private Pose p2_c1 = new Pose(60.000, 42.000);
    private Pose p2_end = new Pose(12.000, 36.000);
    private double p2_end_h = Math.toRadians(180);

    // Intake 1 to Shoot pose 2
    private Pose p3_c1 = new Pose(56.000, 36.000);
    private double p3_end_h = Math.toRadians(200);

    // Shoot pose 2 to Intake 2
    private Pose p4_c1 = new Pose(60.000, 64.000 + 4.0); // Manual Correction
    private Pose p4_end = new Pose(12.000, 60.000 + 4.0); // Manual Correction
    private double p4_end_h = Math.toRadians(180);

    // Intake 2 to Shoot pose 3
    private Pose p5_c1 = new Pose(60.000, 64.000);
    private Pose p5_end = new Pose(60.000, 84.000);
    private double p5_end_h = Math.toRadians(220);

    // Shoot pose 3 to Intake 3
    private Pose p6_end = new Pose(12.000, 84.000);
    private double p6_end_h = Math.toRadians(180);

    // Park pose far
    private Pose pFar_end = new Pose(32.000, 36.000);
    private double pFar_end_h = Math.toRadians(180);
    private Pose pClose_end = new Pose(55.000, 60.000);
    private double pClose_end_h = Math.toRadians(180);
    // endregion


    // PathChain member variables, to be initialized in the constructor
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8;

    /**
     * The main constructor that builds paths for the specified alliance.
     * @param follower The Follower object from your drive train.
     */
    public FarPaths(Follower follower, boolean isBlueSide, MainFarAutonomous.Species species) {
        // If RED alliance, reflect all master poses and headings.
        // If BLUE, this block is skipped and the default values are used.
        if (!isBlueSide) {
            p1_c1 = reflect(p1_c1);
            p1_start = reflect(p1_start);
            p1_end = reflect(p1_end);
            p1_start_h = reflect(p1_start_h);
            p1_end_h = reflect(p1_end_h);

            p2_c1 = reflect(p2_c1);
            p2_end = reflect(p2_end);
            p2_end_h = reflect(p2_end_h);

            p3_c1 = reflect(p3_c1);
            p3_end_h = reflect(p3_end_h);

            p4_c1 = reflect(p4_c1);
            p4_end = reflect(p4_end);
            p4_end_h = reflect(p4_end_h);

            p5_c1 = reflect(p5_c1);
            p5_end = reflect(p5_end);
            p5_end_h = reflect(p5_end_h);

            p6_end = reflect(p6_end);
            p6_end_h = reflect(p6_end_h);

            pFar_end = reflect(pFar_end);
            pFar_end_h = reflect(pFar_end_h);
            pClose_end = reflect(pClose_end);
            pClose_end_h = reflect(pClose_end_h);

            START_POSE = new Pose(p1_start.getX(), p1_start.getY(), p1_start_h);
        }

        switch (species) {
            case SOLO:
            case PUSH:
                // Push alliance out of shooting line
                Path1 = follower.pathBuilder()
                        .addPath(new BezierCurve(p1_start, p1_c1, p1_end))
                        .setLinearHeadingInterpolation(p1_start_h, p1_end_h)
                        .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                        .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                        .build();
                break;
            case SYMBIOTIC:
                // Straight line from START to Shoot pose 1
                Path1 = follower.pathBuilder()
                        .addPath(new BezierLine(p1_start, p1_end))
                        .setLinearHeadingInterpolation(p1_start_h, p1_end_h)
                        .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                        .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                        .build();
                break;
        }

        // Shoot pose 1 to Intake 1
        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(p1_end, p2_c1, p2_end))
                .setLinearHeadingInterpolation(p1_end_h, p2_end_h)
                .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                .build();

        // Intake 1 to Shoot pose 2
        Path3 = follower.pathBuilder()
                .addPath(new BezierCurve(p2_end, p3_c1, p1_end))
                .setLinearHeadingInterpolation(p2_end_h, p3_end_h)
                .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                .build();

        // Shoot pose 2 to Intake 2
        Path4 = follower.pathBuilder()
                .addPath(new BezierCurve(p1_end, p4_c1, p4_end))
                .setLinearHeadingInterpolation(p3_end_h, p4_end_h)
                .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                .build();

        switch (species) {
            case SOLO:
                // Intake 2 to Shoot Pose 3
                Path5 = follower.pathBuilder()
                        .addPath(new BezierCurve(p4_end, p5_c1, p5_end))
                        .setLinearHeadingInterpolation(p4_end_h, p5_end_h)
                        .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                        .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                        .build();

                // Shoot pose 3 to Intake 3
                Path6 = follower.pathBuilder()
                        .addPath(new BezierLine(p5_end, p6_end))
                        .setLinearHeadingInterpolation(p5_end_h, p6_end_h)
                        .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                        .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                        .build();

                // Intake 3 to Shoot Pose 3
                Path7 = follower.pathBuilder()
                        .addPath(new BezierLine(p6_end, p5_end)) // This is the reverse of Path 6
                        .setLinearHeadingInterpolation(p6_end_h, p5_end_h)
                        .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                        .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                        .build();

                // Shoot Pose 3 to PARK
                Path8 = follower.pathBuilder()
                        .addPath(new BezierLine(p5_end, pClose_end))
                        .setLinearHeadingInterpolation(p5_end_h, pClose_end_h)
                        .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                        .setVelocityConstraint(PARKING_VELOCITY_CONSTRAINT)
                        .build();

                break;
            case PUSH:
            case SYMBIOTIC:
                // Intake 2 to default Shoot Pose 1
                Path5 = follower.pathBuilder()
                        .addPath(new BezierLine(p4_end, p1_end))
                        .setLinearHeadingInterpolation(p4_end_h, p1_end_h)
                        .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                        .setVelocityConstraint(DEFAULT_VELOCITY_CONSTRAINT)
                        .build();

                // Shoot Pose 3 to PARK
                Path6 = follower.pathBuilder()
                        .addPath(new BezierLine(p5_end, pFar_end))
                        .setLinearHeadingInterpolation(p5_end_h, pFar_end_h)
                        .setBrakingStrength(DEFAULT_BRAKING_STRENGTH)
                        .setVelocityConstraint(PARKING_VELOCITY_CONSTRAINT)
                        .build();

                break;
        }
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
