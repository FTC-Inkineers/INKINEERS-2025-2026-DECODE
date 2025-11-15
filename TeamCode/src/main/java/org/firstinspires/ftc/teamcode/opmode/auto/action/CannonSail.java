package org.firstinspires.ftc.teamcode.opmode.auto.action;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.BOTH;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.LEFT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeUnitState.INTAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.VisionSubsystem;

/**
 * An action to autonomously shoot three balls.
 * This action will rev up the shooter, wait until it's at speed, fire, and then stop.
 */
@Config
public abstract class CannonSail implements Sail {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private VisionSubsystem.ObeliskMotif motif;
    private final ElapsedTime timer = new ElapsedTime();

    protected abstract double rpm();

    // Action phases
    private enum ShootState {
        IDLE,
        RAMP_UP,
        FIRE_1,
        FIRE_2,
        FIRE_3,
        DONE
    }
    private ShootState currentState = ShootState.IDLE;
    private int index;

    public static double SHOOT_INTERVAL_TIMEOUT = 1.8;
    public static double SHOOTER_RAMP_UP_TIMEOUT = 1.8;

    public CannonSail(ShooterSubsystem shooter, IntakeSubsystem intake, VisionSubsystem.ObeliskMotif motif, int index) {
        this.shooter = shooter;
        this.intake = intake;

        // What cycle it's on
        this.index = index;
        // Default to PGP if UNKNOWN is somehow passed
        this.motif = (motif == VisionSubsystem.ObeliskMotif.UNKNOWN) ? VisionSubsystem.ObeliskMotif.PGP : motif;
    }

    /*
    Motif based control: ALWAYS LOAD GREEN ON LEFT SIDE
    GPP: trigger(REVERSE) ~ right + trigger(FORWARD) ~ left + trigger(FORWARD) ~ trigger(FORWARD)
    PGP: trigger(FORWARD) ~ left + trigger(FORWARD) ~ right + trigger(FORWARD)
    PPG: trigger(FORWARD) ~ right + trigger(FORWARD) ~ left + trigger(FORWARD)
    UNKNOWN: same as PGP
     */

    @Override
    public void initialize() {
        intake.stop(); // Stops intaking after picking up 3 balls
        // Start the process by revving up the shooter and slowly intake
        shooter.setTargetRPM(rpm());
        intake.setPower(0.5);
        currentState = ShootState.RAMP_UP;
        timer.reset();
    }

    private boolean greenOnLeft = true;
    @Override
    public void execute() {
        // Always run shooter PID
        shooter.updateShooterPower();

        // State machine for the shooting sequence
        switch (currentState) {
            case RAMP_UP:
                switch (motif) {
                    case GPP: // Store purple
                        shooter.reverseTrigger();
                        break;
                    case PGP:
                    case PPG:
                        break;
                }
                // Wait for the shooter to reach its target RPM or timeout
                if (shooter.isReady() || timer.seconds() > SHOOTER_RAMP_UP_TIMEOUT) {
                    intake.setPower(0.96); // Back to default power
                    intake.stop();
                    shooter.releaseTrigger();
                    currentState = ShootState.FIRE_1;
                    timer.reset();
                }
                break;

            case FIRE_1:
                // First shot logic based on the detected motif

                switch (motif) {
                    case GPP: // Green is first: give time to fire left
                        if (timer.seconds() > 0.8) {
                            intake.stop();
                            shooter.pullTrigger();
                        } else {
                            intake.setIntake(greenOnLeft ? LEFT : RIGHT, INTAKE);
                        }
                        break;
                    case PGP: // Purple is first: FORWARD trigger
                    case PPG:
                        shooter.pullTrigger();
                        break;
                }

                if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                    shooter.setTargetRPM(rpm()); // Go back to default RPM
                    intake.stop(); // Pause all unused intakes for next shot
                    currentState = ShootState.FIRE_2;
                    timer.reset();
                }
                break;

            case FIRE_2:
                // Second shot logic
                shooter.pullTrigger(); // Always trigger forward for 2nd and 3rd shots
                switch (motif) {
                    case GPP: // Second is Purple: fire stored element
                        break;
                    case PGP: // Second is Green: fire LEFT element
                        intake.setIntake(greenOnLeft ? LEFT : RIGHT, INTAKE);
                        break;
                    case PPG: // Second is Purple: fire RIGHT element
                        intake.setIntake(greenOnLeft ? RIGHT : LEFT, INTAKE);
                        break;
                }

                //
                if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT - 1.0) {
                    intake.stop(); // Pause all unused intakes for next shot
                    currentState = ShootState.FIRE_3;
                    timer.reset();
                }
                break;

            case FIRE_3:
                // Third shot logic
                shooter.pullTrigger(); // Always trigger forward
                // The last element is always fed by both intakes to ensure it fires
                intake.setIntake(BOTH, INTAKE);

                if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                    intake.stop();
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
        intake.stop();
        shooter.setTargetRPM(0);
        shooter.windDown();
        shooter.releaseTrigger();
    }
}
