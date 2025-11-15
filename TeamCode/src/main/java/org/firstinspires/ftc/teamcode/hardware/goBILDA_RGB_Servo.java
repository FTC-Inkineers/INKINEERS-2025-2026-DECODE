package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Represents a goBILDA RGB LED controlled by a Servo.
 * This class abstracts the PWM values into named colors for easy use.
 */
public class goBILDA_RGB_Servo {

    private final Servo rgbServo;

    // Color Scale constants (PWM values for the servo)
    public static final double BLACK = 0.100;
    public static final double RED = 0.288;
    public static final double ORANGE = 0.333;
    public static final double YELLOW = 0.388;
    public static final double SAGE = 0.444;
    public static final double GREEN = 0.500;
    public static final double AZURE = 0.555;
    public static final double BLUE = 0.611;
    public static final double INDIGO = 0.666;
    public static final double VIOLET = 0.722;
    public static final double WHITE = 0.900;

    /**
     * Constructor for the goBILDA_RGB light strip.
     * @param hardwareMap The robot's hardware map.
     * @param deviceName The configured name of the servo in the robot controller.
     */
    public goBILDA_RGB_Servo(HardwareMap hardwareMap, String deviceName) {
        rgbServo = hardwareMap.get(Servo.class, deviceName);
    }

    /**
     * Sets the color of the LED by providing a raw PWM value.
     * @param pwmValue The servo position (0.0 to 1.0) corresponding to a color.
     */
    public void setPosition(double pwmValue) {
        // Basic safety clamp to ensure the value is valid for a servo.
        if (pwmValue < 0.0) pwmValue = 0.0;
        if (pwmValue > 1.0) pwmValue = 1.0;
        rgbServo.setPosition(pwmValue);
    }
}
