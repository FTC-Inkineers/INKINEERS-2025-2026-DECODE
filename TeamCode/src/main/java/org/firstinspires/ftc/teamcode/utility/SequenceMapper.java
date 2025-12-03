package org.firstinspires.ftc.teamcode.utility;

import java.util.ArrayList;
import java.util.List;

/**
 * Helper class for FIRST Tech Challenge (FTC) to determine the firing sequence
 * (LEFT, MIDDLE, RIGHT) required to transform a physical ball configuration
 * (Config) into a desired target sequence (Desired).
 *
 * This class is designed to be used by an OpMode.
 */
public class SequenceMapper {

    /**
     * Enum representing the 3 possible ball configurations (physical layout or target sequence).
     */
    public enum PositionConfig {
        GPP(new char[]{'G', 'P', 'P'}), // Green at Left (1), Purples at Middle (2) and Right (3)
        PGP(new char[]{'P', 'G', 'P'}), // Green at Middle (2), Purples at Left (1) and Right (3)
        PPG(new char[]{'P', 'P', 'G'}); // Green at Right (3), Purples at Left (1) and Middle (2)

        private final char[] array;

        PositionConfig(char[] array) {
            this.array = array;
        }

        public char[] getArray() {
            return array;
        }

        /**
         * Optional utility to get a config by a simple integer ID (1, 2, or 3).
         */
        public static PositionConfig fromId(int id) {
            switch (id) {
                case 1: return GPP;
                case 2: return PGP;
                case 3: return PPG;
                default: throw new IllegalArgumentException("ID must be 1, 2, or 3.");
            }
        }
    }

    /**
     * Enum representing the 6 possible output sequences (the firing plan).
     * L=LEFT(1), M=MIDDLE(2), R=RIGHT(3)
     */
    public enum Sequence {
        LMR, // Left, Middle, Right [1, 2, 3]
        LRM, // Left, Right, Middle [1, 3, 2]
        MLR, // Middle, Left, Right [2, 1, 3]
        MRL, // Middle, Right, Left [2, 3, 1]
        RLM, // Right, Left, Middle [3, 1, 2]
        RML; // Right, Middle, Left [3, 2, 1]

        /**
         * Converts a result array (e.g., [2, 1, 3]) into the matching enum (e.g., MLR).
         */
        public static Sequence fromIndices(int[] indices) {
            if (indices.length != 3) {
                // This shouldn't happen if called correctly by the mapper logic
                throw new IllegalArgumentException("Indices array must have length 3.");
            }

            // Maps the first two indices to the unique sequence enum
            if (indices[0] == 1) { // L is first
                return (indices[1] == 2) ? LMR : LRM;
            } else if (indices[0] == 2) { // M is first
                return (indices[1] == 1) ? MLR : MRL;
            } else { // R is first (indices[0] == 3)
                return (indices[1] == 1) ? RLM : RML;
            }
        }
    }

    /**
     * UTILITY: Returns the mirrored equivalent of a PositionConfig for asymmetrical fields
     * (e.g., swapping GPP and PPG when switching from Blue to Red Alliance).
     *
     * @param config The pattern detected on the field.
     * @return The mirrored pattern.
     */
    public static PositionConfig getMirroredConfig(PositionConfig config) {
        switch (config) {
            case GPP: return PositionConfig.PPG; // GPP mirrors to PPG
            case PPG: return PositionConfig.GPP; // PPG mirrors to GPP
            case PGP: return PositionConfig.PGP; // PGP is symmetrical, so it mirrors to itself
            default: return config; // Should not happen
        }
    }

    /**
     * CORE LOGIC: Finds the 1-based indices from config to match the desired sequence.
     * This method handles the low-level array manipulation.
     *
     * @param config  A 3-element char array representing the starting state.
     * @param desired A 3-element char array representing the desired end state.
     * @return A 3-element int array of the 1-based indices (1=LEFT, 2=MIDDLE, 3=RIGHT)
     * from config to build the desired array.
     */
    private int[] getMappingSequence(char[] config, char[] desired) {
        if (config.length != 3 || desired.length != 3) {
            throw new IllegalArgumentException("Input arrays must both be of length 3.");
        }

        int[] result = new int[3];
        int greenIndex = -1;
        List<Integer> purpleIndices = new ArrayList<>();

        // 1. Find the 0-based indices of 'G' and 'P's in the config array (Inventory Check)
        for (int i = 0; i < config.length; i++) {
            if (config[i] == 'G') {
                greenIndex = i;
            } else if (config[i] == 'P') {
                purpleIndices.add(i);
            }
        }

        if (greenIndex == -1 || purpleIndices.size() != 2) {
            // Safety check against invalid input data
            throw new IllegalArgumentException("Config must contain exactly 1 'G' and 2 'P's.");
        }

        // 2. Build the result array by mapping desired elements to config indices (Plan Generation)
        int purpleCounter = 0;
        for (int i = 0; i < desired.length; i++) {
            if (desired[i] == 'G') {
                // Map the Green ball's physical location (0-based) to the 1-based index (1, 2, 3)
                result[i] = greenIndex + 1;
            } else if (desired[i] == 'P') {
                // Map the next available Purple ball's physical location (0-based)
                // This implements the deterministic rule: always use the lowest-numbered physical slot first.
                result[i] = purpleIndices.get(purpleCounter) + 1;
                purpleCounter++; // Consume one of the available Purple indices
            }
        }

        return result;
    }

    /**
     * PUBLIC API: High-level function that provides the firing plan using enums.
     *
     * @param desiredCombination The detected target sequence (e.g., PositionConfig.PGP).
     * @param currentConfiguration The robot's physical setup (e.g., PositionConfig.GPP).
     * @return The resulting Sequence enum (e.g., Sequence.MLR), which is the firing plan.
     */
    public Sequence getMappedSequenceEnum(PositionConfig desiredCombination, PositionConfig currentConfiguration) {
        // Prepare the inputs for the core logic
        char[] configArr = currentConfiguration.getArray();
        char[] desiredArr = desiredCombination.getArray();

        // Get the index array (the plan)
        int[] indices = getMappingSequence(configArr, desiredArr);

        // Convert the index array (e.g., [2, 1, 3]) to the enum (e.g., MLR)
        return Sequence.fromIndices(indices);
    }
}
