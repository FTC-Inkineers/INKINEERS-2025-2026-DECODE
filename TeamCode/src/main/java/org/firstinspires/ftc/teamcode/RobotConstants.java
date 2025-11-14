package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;

@Config
@Configurable
public class RobotConstants {
    // ODOMETRY
    public static final double xVelocity = 76.0;
    public static final double yVelocity = 60.0;
    public static final double forwardZeroPowerAcceleration = -41.0;
    public static final double lateralZeroPowerAcceleration = -86.0;
    public static double DEFAULT_BRAKING_STRENGTH = 0.8;
    public static double DEFAULT_VELOCITY_CONSTRAINT = 30;

    public static FilteredPIDFCoefficients drive = new FilteredPIDFCoefficients(
            0.025,
            0,
            0.00001,
            0.6,
            0.01);
    public static PIDFCoefficients translational = new PIDFCoefficients(
            0.1,
            0,
            0,
            0.015);
    public static PIDFCoefficients heading = new PIDFCoefficients(
            1,
            0,
            0,
            0.01);

    // SHOOTER
    public static final double MAX_FLYWHEEL_RPM = 6000;
}
