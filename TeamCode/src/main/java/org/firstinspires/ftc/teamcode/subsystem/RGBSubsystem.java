package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.RobotConstants.MAX_FLYWHEEL_RPM;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.goBILDA_RGB_Continuous;

public class RGBSubsystem {
    private final goBILDA_RGB_Continuous rgb; // Use the new goBILDA_RGB object
    private double currentColor;

    private final ElapsedTime pulseTimer = new ElapsedTime();
    private static final double PULSE_AMPLITUDE = 0.3; // How far from the base color it shifts (+/- 0.3)
    private static final double PULSE_SPEED = 3.0;

    public RGBSubsystem(HardwareMap hardwareMap) {
        rgb = new goBILDA_RGB_Continuous(hardwareMap, "RGB");
        currentColor = goBILDA_RGB_Continuous.GREEN;
    }

    public void resetPulseTimer() {
        pulseTimer.reset();
    }

    public void runTeleOp(DriveSubsystem.DriveState driveState, boolean lockedOn) {
        switch (driveState) {
            case MANUAL:
                setColor(goBILDA_RGB_Continuous.GREEN);
                break;
            case MANUAL_AIM_ASSIST:
                if (lockedOn) {
                    setColor(goBILDA_RGB_Continuous.RED);
                } else {
                    setColor(goBILDA_RGB_Continuous.GREEN);
                    pulse();
                }
                break;
            case HOLD_POSITION:
                setColor(goBILDA_RGB_Continuous.VIOLET);
                break;
        }
        rgb.setPower(currentColor);
    }

    public void runAuto(double rpm) {
        rgb.setPower(rpm / MAX_FLYWHEEL_RPM - 0.1);
    }

    /**
     * Sets the base color for the subsystem to manage.
     * @param pwm The color's PWM value.
     */
    public void setColor(double pwm) {
        this.currentColor = pwm;
    }

    public void pulse() {
        // Calculate the sine wave's current value. It oscillates between -1 and 1.
        double sineOutput = Math.sin(pulseTimer.seconds() * PULSE_SPEED);

        // Scale the sine wave's output by our desired amplitude (+/- 0.3)
        double offset = sineOutput * PULSE_AMPLITUDE;

        // Apply the smooth, oscillating offset to the base color
        rgb.setPower(currentColor + offset);
    }
}
