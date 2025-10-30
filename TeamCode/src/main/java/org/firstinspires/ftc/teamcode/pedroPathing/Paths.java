package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    // Enum to easily select the alliance side in your OpModes
    public enum Alliance {
        BLUE, // The paths as defined below
        RED   // The reflected paths
    }

    //region MASTER PATH DEFINITIONS (Default to BLUE alliance)
    // These variables will be used as-is for BLUE or reflected for RED.

    private Pose p1_start = new Pose(56.000, 8.000);
    private Pose p1_end   = new Pose(56.000, 17.000);
    private double p1_start_h = Math.toRadians(90);
    private double p1_end_h   = Math.toRadians(120);

    private Pose p2_c1 = new Pose(60.000, 42.000);
    private Pose p2_end = new Pose(12.000, 36.000);
    private double p2_end_h = Math.toRadians(90);

    private Pose p3_c1 = new Pose(56.000, 36.000);
    private Pose p3_end = new Pose(44.000, 64.000);
    private double p3_end_h = Math.toRadians(120);

    private Pose p4_end = new Pose(12.000, 60.000);
    private double p4_end_h = Math.toRadians(90);

    private Pose p5_c1 = new Pose(56.000, 64.000);
    private Pose p5_end = new Pose(56.000, 84.000);
    private double p5_end_h = Math.toRadians(130);

    private Pose p6_end = new Pose(12.000, 84.000);
    private double p6_end_h = Math.toRadians(90);
    //endregion

    // PathChain member variables, to be initialized in the constructor
    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

    /**
     * Default constructor, assumes BLUE alliance.
     * @param follower The Follower object from your drive train.
     */
    public Paths(Follower follower) {
        this(follower, Alliance.BLUE); // Defaults to BLUE
    }

    /**
     * The main constructor that builds paths for the specified alliance.
     * @param follower The Follower object from your drive train.
     * @param alliance The alliance color (BLUE or RED).
     */
    public Paths(Follower follower, Alliance alliance) {
        // If RED alliance, reflect all master poses and headings.
        // If BLUE, this block is skipped and the default values are used.
        if (alliance == Alliance.RED) {
            p1_start = reflect(p1_start);
            p1_end = reflect(p1_end);
            p1_start_h = reflect(p1_start_h);
            p1_end_h = reflect(p1_end_h);

            p2_c1 = reflect(p2_c1);
            p2_end = reflect(p2_end);
            p2_end_h = reflect(p2_end_h);

            p3_c1 = reflect(p3_c1);
            p3_end = reflect(p3_end);
            p3_end_h = reflect(p3_end_h);

            p4_end = reflect(p4_end);
            p4_end_h = reflect(p4_end_h);

            p5_c1 = reflect(p5_c1);
            p5_end = reflect(p5_end);
            p5_end_h = reflect(p5_end_h);

            p6_end = reflect(p6_end);
            p6_end_h = reflect(p6_end_h);
        }

        // Now, build the PathChain objects using the (potentially reflected) class member variables.
        // Path start/end points are linked dynamically.
        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(p1_start, p1_end))
                .setLinearHeadingInterpolation(p1_start_h, p1_end_h)
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(p1_end, p2_c1, p2_end))
                .setLinearHeadingInterpolation(p1_end_h, p2_end_h)
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierCurve(p2_end, p3_c1, p3_end))
                .setLinearHeadingInterpolation(p2_end_h, p3_end_h)
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(p3_end, p4_end))
                .setLinearHeadingInterpolation(p3_end_h, p4_end_h)
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierCurve(p4_end, p5_c1, p5_end))
                .setLinearHeadingInterpolation(p4_end_h, p5_end_h)
                .build();

        Path6 = follower.pathBuilder()
                .addPath(new BezierLine(p5_end, p6_end))
                .setLinearHeadingInterpolation(p5_end_h, p6_end_h)
                .build();

        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(p6_end, p5_end)) // This is the reverse of Path 6
                .setLinearHeadingInterpolation(p6_end_h, p5_end_h)
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
