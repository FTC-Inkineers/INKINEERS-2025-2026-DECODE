package org.firstinspires.ftc.teamcode.opmode.auto.action;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/**
 * An action to autonomously shoot one or more rings.
 * This action will rev up the shooter, wait until it's at speed, fire, and then stop.
 */
public class ShootAction implements Action {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final ElapsedTime timer = new ElapsedTime();

    // Action phases
    private enum ShootState {
        IDLE,
        RAMP_UP,
        FIRE,
        DONE
    }
    private ShootState currentState = ShootState.IDLE;

    private final double SHOOTER_RAMP_UP_TIMEOUT_S = 2.0;
    private final double FIRING_DURATION_S = 1.0;

    public ShootAction(ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        // Start the process by revving up the shooter
        shooter.setTargetRPM(shooter.getStationaryRPM());
        currentState = ShootState.RAMP_UP;
        timer.reset();
    }

    @Override
    public void execute() {
        // This is a state machine within the action
        switch (currentState) {
            case RAMP_UP:
                // Wait for the shooter to reach its target RPM
                shooter.updateShooterPower();
                if (shooter.isAtTargetRPM() || timer.seconds() > SHOOTER_RAMP_UP_TIMEOUT_S) {
                    // Once at speed (or timed out), move to the firing state
                    currentState = ShootState.FIRE;
                    timer.reset();
                }
                break;
            case FIRE:
                // Run the trigger motor to push the ring into the flywheel
                shooter.pullTrigger();
                if (timer.seconds() > FIRING_DURATION_S) {
                    // Firing is complete
                    currentState = ShootState.DONE;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        // The action is done when the internal state machine reaches DONE
        return currentState == ShootState.DONE;
    }

    @Override
    public void end() {
        // Cleanup: Stop the shooter and trigger motors
        shooter.setTargetRPM(0);
        shooter.updateShooterPower();
        shooter.releaseTrigger();
    }
}
