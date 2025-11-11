package org.firstinspires.ftc.teamcode.opmode.auto.action;

import java.util.Arrays;
import java.util.List;

/**
 * An action that runs multiple actions at the same time.
 * This action is considered finished only when all of its child actions are finished.
 */
public class ParallelAction implements Action {
    private final List<Action> actions;

    public ParallelAction(Action... actions) {
        this.actions = Arrays.asList(actions);
    }

    @Override
    public void initialize() {
        // Initialize all child actions
        for (Action action : actions) {
            action.initialize();
        }
    }

    @Override
    public void execute() {
        // Execute all child actions that are not yet finished
        for (Action action : actions) {
            if (!action.isFinished()) {
                action.execute();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // The parallel action is finished only if all child actions are finished
        for (Action action : actions) {
            if (!action.isFinished()) {
                return false; // At least one action is still running
            }
        }
        return true; // All actions are finished
    }

    @Override
    public void end() {
        // Call the end method for all child actions
        for (Action action : actions) {
            action.end();
        }
    }
}
