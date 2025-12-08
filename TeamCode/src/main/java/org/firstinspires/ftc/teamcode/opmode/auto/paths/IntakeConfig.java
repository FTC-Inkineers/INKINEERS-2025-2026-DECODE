package org.firstinspires.ftc.teamcode.opmode.auto.paths;

public class IntakeConfig {
    
    public enum Pattern {
        PRELOAD,
        FAR,
        MIDDLE,
        CLOSE,
        CORNER
    }
    
    public final int patternIndex;       // The specific intake pattern (0-4)
    public final double speed;           // Speed for the path TO this sample
    
    public IntakeConfig(int patternIndex) {
        this(patternIndex, 0.6); // Default speed
    }

    public IntakeConfig(int patternIndex, double speed) {
        this.patternIndex = patternIndex;
        this.speed = speed;
    }

    // Define the Intake Configurations here
    public static final IntakeConfig PRELOAD = new IntakeConfig(0);
    public static final IntakeConfig FAR = new IntakeConfig(3);
    public static final IntakeConfig MIDDLE = new IntakeConfig(2);
    public static final IntakeConfig CLOSE = new IntakeConfig(1);
    public static final IntakeConfig CORNER = new IntakeConfig(4, 0.35); // Specific speed for this one
}
