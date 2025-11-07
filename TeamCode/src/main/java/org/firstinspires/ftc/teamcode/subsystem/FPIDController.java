package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FPIDController {
    private final double kF;
    private final double kP;
    private final double kI;
    private final double kD;

    // State variables
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;

    /**
     * Private constructor that takes the Builder as an argument.
     * This is the only way to create an FPIDController instance.
     */
    private FPIDController(Builder builder) {
        this.kP = builder.kP;
        this.kI = builder.kI;
        this.kD = builder.kD;
        this.kF = builder.kF;
    }

    // --- Static Builder Class ---
    public static class Builder {
        // P is required, so it's final and set in the constructor
        private final double kP;
        // Other gains are optional and have default values
        private double kI = 0;
        private double kD = 0;
        private double kF = 0;

        public Builder(double kP) {
            this.kP = kP;
        }

        public Builder withI(double kI) {
            this.kI = kI;
            return this;
        }

        public Builder withD(double kD) {
            this.kD = kD;
            return this;
        }

        public Builder withF(double kF) {
            this.kF = kF;
            return this;
        }

        /**
         * Creates and returns a new FPIDController instance with the configured gains.
         * @return A new FPIDController.
         */
        public FPIDController build() {
            return new FPIDController(this);
        }
    }

    /**
     * Calculates the control output with a feedforward term based on the target.
     * Ideal for velocity control (like shooters).
     * @param currentError The current error value (target - actual).
     * @param target The desired target value (used for the feedforward term).
     * @return An FPIDOutput object containing all components of the calculation.
     */
    public FPIDOutput calculate(double currentError, double target) {
        double elapsedTime = timer.seconds();

        // F term: Proportional to the target, providing a baseline power
        double f = target * kF;

        // P term: Proportional to the current error
        double p = currentError * kP;

        // I term: Accumulates error over time to correct for steady-state error
        integralSum += currentError * elapsedTime;
        double i = integralSum * kI;

        // D term: Responds to the rate of change of the error (damping)
        double derivative = (elapsedTime > 0) ? (currentError - lastError) / elapsedTime : 0;
        double d = derivative * kD;

        // Reset for the next loop
        lastError = currentError;
        timer.reset();

        // Return the full output object
        return new FPIDOutput(f, p, i, d);
    }

    /**
     * Calculates the control output without a feedforward term.
     * Ideal for position control (like drivetrain alignment).
     * This calls the main calculate method with a target of 0, so the F term is zero.
     * @param currentError The current error value (e.g., target - actual).
     * @return An FPIDOutput object containing all components of the calculation.
     */
    public FPIDOutput calculate(double currentError) {
        // Call the main calculate method with a target of 0
        return calculate(currentError, 0);
    }

    /**
     * Resets the controller's internal state.
     * Call this when the controller is not in use.
     */
    public void reset() {
        lastError = 0;
        integralSum = 0;
        timer.reset();
    }
}
