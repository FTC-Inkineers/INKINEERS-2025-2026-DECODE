package org.firstinspires.ftc.teamcode.opmode.auto.action;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeUnitState.IDLE;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeUnitState.INTAKE;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

/**
 * An action to autonomously shoot one or more rings.
 * This action will rev up the shooter, wait until it's at speed, fire, and then stop.
 */
public class CannonSail implements Sail {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final VisionSubsystem.ObeliskMotif motif;
    private final ElapsedTime timer = new ElapsedTime();

    // Action phases
    private enum ShootState {
        IDLE,
        RAMP_UP,
        FIRE_MID,
        FIRE_LEFT,
        FIRE_RIGHT,
        DONE
    }
    private ShootState currentState = ShootState.IDLE;

    private final double SHOOTER_RAMP_UP_TIMEOUT = 1.5;

    public CannonSail(ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem.ObeliskMotif motif) {
        this.shooter = shooter;
        this.intake = intake;
        this.motif = motif;
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
        // Always run shooter PID
        shooter.updateShooterPower();
        // This is a state machine within the action
        switch (currentState) {
            case RAMP_UP:
                // Wait for the shooter to reach its target RPM
                if (shooter.isReady() || timer.seconds() > SHOOTER_RAMP_UP_TIMEOUT) {
                    // Once at speed (or timed out), move to the firing state
                    currentState = ShootState.FIRE_MID;
                    timer.reset();
                }
                break;
            case FIRE_MID:
                // Run the trigger motor to push the ball into the flywheel
                shooter.pullTrigger();
                intake.setLeftIntakeState(INTAKE);
                intake.setRightIntakeState(IDLE);
                if (timer.seconds() > 1.8) {
                    // Firing is complete
                    currentState = ShootState.FIRE_LEFT;
                    timer.reset();
                }
                break;
            case FIRE_LEFT:
                // Run the trigger motor to push the ball into the flywheel
                shooter.pullTrigger();
                intake.setLeftIntakeState(INTAKE);
                intake.setRightIntakeState(INTAKE);
                if (timer.seconds() > 0.2) {
                    // Firing is complete
                    currentState = ShootState.FIRE_RIGHT;
                    timer.reset();
                }
                break;
            case FIRE_RIGHT:
                // Run the trigger motor to push the ring into the flywheel
                intake.setLeftIntakeState(IDLE);
                intake.setRightIntakeState(INTAKE);
                shooter.pullTrigger();
                if (timer.seconds() > 1.0) {
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
        intake.setRightIntakeState(IDLE);
        intake.setLeftIntakeState(IDLE);
        shooter.setTargetRPM(0);
        shooter.windDown();
        shooter.releaseTrigger();
    }
}
