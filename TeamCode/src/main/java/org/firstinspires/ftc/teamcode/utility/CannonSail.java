    package org.firstinspires.ftc.teamcode.utility;

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
//        private final VisionSubsystem.ObeliskMotif motif;
        private final ElapsedTime timer = new ElapsedTime();

        protected abstract double targetRpm();

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

        public static double SHOOT_INTERVAL_TIMEOUT = 1.2;
        public static double SHOOTER_RAMP_UP_TIMEOUT = 1.8;

        public CannonSail(ShooterSubsystem shooter, IntakeSubsystem intake, SequenceMapper.Sequence sequence, int index) {
            this.shooter = shooter;
            this.intake = intake;

            // What cycle it's on
            this.index = index;
//            // Default to PGP if UNKNOWN is somehow passed
//            this.motif = (motif == VisionSubsystem.ObeliskMotif.UNKNOWN) ? VisionSubsystem.ObeliskMotif.PGP : motif;
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
            shooter.setTargetRPM(targetRpm());
            currentState = ShootState.RAMP_UP;
            timer.reset();
        }

        @Override
        public void execute() {
            // Always run shooter PID
            shooter.updateShooterPhysics();

            runRLM();
        }

        @Override
        public boolean isFinished() {
            // The action is done when the internal state machine reaches DONE
            return currentState == ShootState.DONE;
        }

        @Override
        public void end() {
            intake.stop();
            shooter.stop();
        }

        protected void prepareNextShot() {
            intake.stop();
            shooter.releaseTrigger();
            shooter.setTargetRPM(targetRpm());
            timer.reset();
        }

        public void runLMR() {

        }

        public void runLRM() {

        }

        public void runMLR() {

        }

        public void runMRL() {

        }

        public void runRLM() {
            switch (currentState) {
                case RAMP_UP:
                    // Store Middle Element
                    shooter.reverseTrigger();
                    // Wait for the shooter to reach its target RPM or timeout
                    if (shooter.isReady() || timer.seconds() > SHOOTER_RAMP_UP_TIMEOUT) {
                        prepareNextShot();
                        currentState = ShootState.FIRE_1;
                    }
                    break;
                case FIRE_1:
                    // Intake and Shoot Right Element
                    if (timer.seconds() > 0.8) {
                        intake.stop();
                        shooter.pullTrigger();
                    } else {
                        intake.setIntake(RIGHT, INTAKE);
                    }
                    // Next Shot
                    if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                        prepareNextShot();
                        currentState = ShootState.FIRE_2;
                    }
                    break;
                case FIRE_2:
                    // Shoot Left Element
                    if (timer.seconds() > 0.8) {
                        intake.stop();
                        shooter.pullTrigger();
                    } else {
                        intake.setIntake(LEFT, INTAKE);
                        shooter.reverseTrigger();
                    }
                    // Next Shot
                    if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                        prepareNextShot();
                        currentState = ShootState.FIRE_3;
                    }
                    break;
                case FIRE_3:
                    // Shoot Middle Element
                    shooter.pullTrigger();
                    // Done
                    if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                        intake.stop();
                        currentState = ShootState.DONE;
                    }
                    break;
            }
        }

        public void runRML() {
            switch (currentState) {
                case RAMP_UP:
                    // Store Middle Element
                    shooter.reverseTrigger();
                    // Wait for the shooter to reach its target RPM or timeout
                    if (shooter.isReady() || timer.seconds() > SHOOTER_RAMP_UP_TIMEOUT) {
                        prepareNextShot();
                        currentState = ShootState.FIRE_1;
                    }
                    break;
                case FIRE_1:
                    // Intake and Shoot Right Element
                    if (timer.seconds() > 0.8) {
                        intake.stop();
                        shooter.pullTrigger();
                    } else {
                        intake.setIntake(RIGHT, INTAKE);
                    }
                    // Next Shot
                    if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                        prepareNextShot();
                        currentState = ShootState.FIRE_2;
                    }
                    break;
                case FIRE_2:
                    // Shoot Middle Element
                    shooter.pullTrigger();
                    // Next Shot
                    if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                        prepareNextShot();
                        currentState = ShootState.FIRE_3;
                    }
                    break;
                case FIRE_3:
                    // Shoot Left Element
                    shooter.pullTrigger();
                    intake.setIntake(LEFT, INTAKE);
                    // Done
                    if (timer.seconds() > SHOOT_INTERVAL_TIMEOUT) {
                        intake.stop();
                        currentState = ShootState.DONE;
                    }
                    break;
            }
        }
    }