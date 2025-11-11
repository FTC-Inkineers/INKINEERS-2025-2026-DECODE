package org.firstinspires.ftc.teamcode.opmode.auto.action;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.Queue;

/**
 * An action that runs a series of other actions in sequence.
 * It moves to the next action only when the current one is finished.
 */
public class SequentialAction implements Action {
    private final Queue<Action> actionQueue;
    private Action currentAction;

    public SequentialAction(Action... actions) {
        this.actionQueue = new LinkedList<>(Arrays.asList(actions));
        this.currentAction = null;
    }

    @Override
    public void initialize() {
        // Start the first action in the sequence
        advanceToNextAction();
    }

    @Override
    public void execute() {
        if (currentAction == null) {
            return; // Nothing to do
        }

        // If the current action is finished, move to the next one
        if (currentAction.isFinished()) {
            currentAction.end();
            advanceToNextAction();
        }

        // If there's still a valid action (and it's not finished), execute it
        if (currentAction != null) {
            currentAction.execute();
        }
    }

    private void advanceToNextAction() {
        // Get the next action from the queue
        currentAction = actionQueue.poll();
        if (currentAction != null) {
            // If there is a new action, initialize it
            currentAction.initialize();
        }
    }

    @Override
    public boolean isFinished() {
        // The entire sequence is finished when there are no more actions to run
        return currentAction == null;
    }

    @Override
    public void end() {
        // Ensure the last action's end method is called if it hasn't been already
        if (currentAction != null) {
            currentAction.end();
            currentAction = null;
        }
    }
}
