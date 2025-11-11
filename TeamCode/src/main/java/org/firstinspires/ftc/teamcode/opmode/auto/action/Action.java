package org.firstinspires.ftc.teamcode.opmode.auto.action;

/**
 * An interface for creating modular, sequential actions in autonomous.
 * Each action has a defined lifecycle: initialize, execute, and a condition to end.
 */
public interface Action {
    /**
     * Called once when the action is first started.
     * Place any setup code here.
     */
    void initialize();

    /**
     * Called repeatedly in the main loop until the action is finished.
     * Place the code that performs the action here.
     */
    void execute();

    /**
     * The condition that determines when the action is complete.
     * @return True when the action is finished, false otherwise.
     */
    boolean isFinished();

    /**
     * Called once after the action is finished.
     * Place any cleanup or finalization code here.
     */
    void end();
}
