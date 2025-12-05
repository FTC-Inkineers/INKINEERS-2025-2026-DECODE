package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_FLYWHEEL_RPM;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.goBILDA_RGB_Servo;

public class RGBSubsystem {
    private final goBILDA_RGB_Servo rgb;
    private double baseColor;

    // Pulse State Variables
    private boolean isPulsing = false;
    private final ElapsedTime pulseTimer = new ElapsedTime();
    private static final double PULSE_AMPLITUDE = 0.05;
    private static final double PULSE_SPEED = 3.0;


    // Flash State Variables
    private boolean isFlashing = false;
    private final ElapsedTime flashTimer = new ElapsedTime();
    private static final double FLASH_DURATION = 0.3; // Seconds
    private static final double FLASH_RATE = 0.1;


    public RGBSubsystem(HardwareMap hardwareMap) {
        rgb = new goBILDA_RGB_Servo(hardwareMap, "RGB");
        baseColor = goBILDA_RGB_Servo.GREEN;
    }

    public void runTeleOp(DriveSubsystem.DriveState driveState, boolean targetVisible, boolean lockedOn, boolean shooterReady) {
        // 1. Determine the Base Behavior (Solid Color vs Pulse)
        isPulsing = false; // Reset default

        switch (driveState) {
            case MANUAL:
                setColor(goBILDA_RGB_Servo.WHITE);
                break;
            case MANUAL_AIM_ASSIST:
                if (lockedOn) {
                    if (shooterReady) {
                        setColor(goBILDA_RGB_Servo.BLUE);
                    } else {
                        setColor(goBILDA_RGB_Servo.GREEN);
                    }
                } else {
                    if (targetVisible) {
                        setColor(goBILDA_RGB_Servo.YELLOW);
                    } else {
                        setColor(goBILDA_RGB_Servo.RED);
                    }
                }
                break;
            case HOLD_POSITION:
                setColor(goBILDA_RGB_Servo.VIOLET);
                break;
        }

        // 2. Apply Effects Priority: Flash > Pulse > Solid
        if (isFlashing) {
            handleFlashEffect();
        } else if (isPulsing) {
            handlePulseEffect();
        } else {
            rgb.setPosition(baseColor);
        }
    }

    public void runAuto(double rpm) {
        rgb.setPosition(rpm / MAX_FLYWHEEL_RPM - 0.1);
    }

    /**
     * Sets the base color.
     */
    public void setColor(double pwm) {
        this.baseColor = pwm;
    }

    /**
     * Flags the system to pulse during this loop cycle.
     */
    public void startPulse() {
        isPulsing = true;
    }

    private void handlePulseEffect() {
        // Calculate sine wave (-1 to 1)
        double sineOutput = Math.sin(pulseTimer.seconds() * PULSE_SPEED);
        // Scale by amplitude
        double offset = sineOutput * PULSE_AMPLITUDE;
        // Apply to base color and clamp
        double finalColor = Math.max(0.289, Math.min(0.898, baseColor + offset));

        rgb.setPosition(finalColor);
    }

    /**
     * Triggers a flash effect for a set duration.
     */
    public void flash() {
        isFlashing = true;
        flashTimer.reset();
    }

    private void handleFlashEffect() {
        if (flashTimer.seconds() > FLASH_DURATION) {
            isFlashing = false; // Time's up
            rgb.setPosition(baseColor); // Revert immediately
        } else {
            // Blink logic
            double remainder = flashTimer.seconds() % (FLASH_RATE * 2);
            if (remainder < FLASH_RATE) {
                rgb.setPosition(0);
            } else {
                rgb.setPosition(baseColor);
            }
        }
    }

}
