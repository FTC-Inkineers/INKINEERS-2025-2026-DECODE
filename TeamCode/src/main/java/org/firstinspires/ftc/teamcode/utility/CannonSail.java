package org.firstinspires.ftc.teamcode.utility;

import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.LEFT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeSide.RIGHT;
import static org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem.IntakeUnitState.INTAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

/**
 * An action to autonomously shoot three balls based on a sequence.
 * This action will rev up the shooter, wait until it's at speed, and execute the firing sequence.
 */
@Config
public abstract class CannonSail implements Sail {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final SequenceMapper.Sequence sequence;
    private Sail sequenceSail;

    public static double SHOOT_INTERVAL_TIMEOUT = 1.2;
    public static double SHOOTER_RAMP_UP_TIMEOUT = 1.8;
    public static double REVERSE_SPINNER_TIME = 0.3;

    public CannonSail(ShooterSubsystem shooter, IntakeSubsystem intake, SequenceMapper.Sequence sequence, int index) {
        this.shooter = shooter;
        this.intake = intake;
        this.sequence = sequence;
    }

    protected abstract double rpm();

    @Override
    public void initialize() {
        double targetRpm = rpm();
        switch (sequence) {
            case LMR:
                sequenceSail = new SequenceLMR(shooter, intake, targetRpm);
                break;
            case LRM:
                sequenceSail = new SequenceLRM(shooter, intake, targetRpm);
                break;
            case MLR:
                sequenceSail = new SequenceMLR(shooter, intake, targetRpm);
                break;
            case MRL:
                sequenceSail = new SequenceMRL(shooter, intake, targetRpm);
                break;
            case RLM:
                sequenceSail = new SequenceRLM(shooter, intake, targetRpm);
                break;
            case RML:
                sequenceSail = new SequenceRML(shooter, intake, targetRpm);
                break;
        }
        if (sequenceSail != null) {
            sequenceSail.initialize();
        }
    }

    @Override
    public void execute() {
        if (sequenceSail != null) {
            sequenceSail.execute();
        } else {
            // Fallback just in case, though sequence should always be set
            shooter.updateShooterPhysics();
        }
    }

    @Override
    public boolean isFinished() {
        return sequenceSail != null && sequenceSail.isFinished();
    }

    @Override
    public void end() {
        if (sequenceSail != null) {
            sequenceSail.end();
        } else {
            // Safety cleanup
            shooter.stop();
            intake.stop();
        }
    }

    // --- Base Sequence Sail ---

    public abstract static class AbstractSequenceSail implements Sail {
        protected final ShooterSubsystem shooter;
        protected final IntakeSubsystem intake;
        protected final double targetRpm;
        protected final ElapsedTime timer = new ElapsedTime();

        protected enum ShootState {
            IDLE, RAMP_UP, FIRE_1, FIRE_2, FIRE_3, DONE
        }

        protected ShootState currentState = ShootState.IDLE;

        public AbstractSequenceSail(ShooterSubsystem shooter, IntakeSubsystem intake, double targetRpm) {
            this.shooter = shooter;
            this.intake = intake;
            this.targetRpm = targetRpm;
        }

        protected abstract char getAction(int step);

        @Override
        public void initialize() {
            intake.stop();
            shooter.setTargetRPM(targetRpm);
            intake.setPower(0.5);
            currentState = ShootState.RAMP_UP;
            timer.reset();
        }

        @Override
        public void execute() {
            shooter.updateShooterPhysics();

            switch (currentState) {
                case RAMP_UP:
                    if (shooter.isReady() || timer.seconds() > SHOOTER_RAMP_UP_TIMEOUT) {
                        intake.setPower(0.96);
                        intake.stop();
                        shooter.releaseTrigger();
                        currentState = ShootState.FIRE_1;
                        timer.reset();
                    }
                    break;

                case FIRE_1:
                    performAction(getAction(1));
                    if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                        prepareNextShot();
                        currentState = ShootState.FIRE_2;
                        timer.reset();
                    }
                    break;

                case FIRE_2:
                    performAction(getAction(2));
                    if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                        prepareNextShot();
                        currentState = ShootState.FIRE_3;
                        timer.reset();
                    }
                    break;

                case FIRE_3:
                    performAction(getAction(3));
                    if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                        intake.stop();
                        currentState = ShootState.DONE;
                    }
                    break;
            }
        }

        protected void performAction(char action) {
            switch (action) {
                case 'L': // Shoot Left: reverse spinner, then spin left intake
                    if (timer.seconds() < REVERSE_SPINNER_TIME) {
                        shooter.reverseTrigger();
                    } else {
                        shooter.releaseTrigger();
                        intake.setIntake(LEFT, INTAKE);
                    }
                    break;
                case 'M': // Shoot Middle: spin trigger
                    shooter.pullTrigger();
                    break;
                case 'R': // Shoot Right: spin right intake
                    intake.setIntake(RIGHT, INTAKE);
                    break;
            }
        }

        protected void prepareNextShot() {
            intake.stop();
            shooter.releaseTrigger();
            shooter.setTargetRPM(targetRpm);
        }

        @Override
        public boolean isFinished() {
            return currentState == ShootState.DONE;
        }

        @Override
        public void end() {
            intake.stop();
            shooter.setTargetRPM(0);
            shooter.windDown();
            shooter.releaseTrigger();
        }
    }

    // --- Sequence Implementations ---

    public static class SequenceLMR extends AbstractSequenceSail {
        public SequenceLMR(ShooterSubsystem s, IntakeSubsystem i, double rpm) { super(s, i, rpm); }
        @Override protected char getAction(int step) {
            if (step == 1) return 'L';
            if (step == 2) return 'M';
            return 'R';
        }
    }

    public static class SequenceLRM extends AbstractSequenceSail {
        public SequenceLRM(ShooterSubsystem s, IntakeSubsystem i, double rpm) { super(s, i, rpm); }
        @Override protected char getAction(int step) {
            if (step == 1) return 'L';
            if (step == 2) return 'R';
            return 'M';
        }
    }

    public static class SequenceMLR extends AbstractSequenceSail {
        public SequenceMLR(ShooterSubsystem s, IntakeSubsystem i, double rpm) { super(s, i, rpm); }
        @Override protected char getAction(int step) {
            if (step == 1) return 'M';
            if (step == 2) return 'L';
            return 'R';
        }
    }

    public static class SequenceMRL extends AbstractSequenceSail {
        public SequenceMRL(ShooterSubsystem s, IntakeSubsystem i, double rpm) { super(s, i, rpm); }
        @Override protected char getAction(int step) {
            if (step == 1) return 'M';
            if (step == 2) return 'R';
            return 'L';
        }
    }

    public static class SequenceRLM extends AbstractSequenceSail {
        public SequenceRLM(ShooterSubsystem s, IntakeSubsystem i, double rpm) { super(s, i, rpm); }
        @Override protected char getAction(int step) {
            if (step == 1) return 'R';
            if (step == 2) return 'L';
            return 'M';
        }
    }

    public static class SequenceRML extends AbstractSequenceSail {
        public SequenceRML(ShooterSubsystem s, IntakeSubsystem i, double rpm) { super(s, i, rpm); }
        @Override protected char getAction(int step) {
            if (step == 1) return 'R';
            if (step == 2) return 'M';
            return 'L';
        }
    }
}
