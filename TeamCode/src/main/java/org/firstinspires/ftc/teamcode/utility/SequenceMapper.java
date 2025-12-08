package org.firstinspires.ftc.teamcode.utility;

public class SequenceMapper {

    public enum Sequence {
        LMR, // Left, Middle, Right
        LRM, // Left, Right, Middle
        MLR, // Middle, Left, Right
        MRL, // Middle, Right, Left
        RLM, // Right, Left, Middle
        RML  // Right, Middle, Left
    }

    public enum PositionConfig {
        GPP, // Green, Purple, Purple
        PGP, // Purple, Green, Purple
        PPG  // Purple, Purple, Green
    }

    /**
     * Determines the shooting sequence based on the target pattern on the wall (Motif)
     * and the current ball configuration in the robot (Config).
     *
     * @param target  The desired pattern on the wall (Motif)
     * @param current The physical arrangement of balls in the robot (Config)
     * @return The Sequence enum representing the firing order
     */
    public Sequence getMappedSequenceEnum(PositionConfig target, PositionConfig current) {

        // --- MOTIF: GPP ---
        if (target == PositionConfig.GPP) {
            if (current == PositionConfig.GPP) {
                return Sequence.LMR; // Matches: L(G), M(P), R(P) -> GPP
            }
            if (current == PositionConfig.PGP) {
                return Sequence.MLR; // Matches: M(G), L(P), R(P) -> GPP (Alternative: MRL)
            }
            if (current == PositionConfig.PPG) {
                return Sequence.RML; // Matches: R(G), M(P), L(P) -> GPP
            }
        }

        // --- MOTIF: PGP ---
        else if (target == PositionConfig.PGP) {
            if (current == PositionConfig.GPP) {
                return Sequence.MLR; // Matches: M(P), L(G), R(P) -> PGP
            }
            if (current == PositionConfig.PGP) {
                return Sequence.LMR; // Matches: L(P), M(G), R(P) -> PGP (Alternative: RML)
            }
            if (current == PositionConfig.PPG) {
                return Sequence.MRL; // Matches: M(P), R(G), L(P) -> PGP
            }
        }

        // --- MOTIF: PPG ---
        else if (target == PositionConfig.PPG) {
            if (current == PositionConfig.GPP) {
                return Sequence.MRL; // Matches: M(P), R(P), L(G) -> PPG
            }
            if (current == PositionConfig.PGP) {
                return Sequence.LRM; // Matches: L(P), R(P), M(G) -> PPG (Alternative: RLM)
            }
            if (current == PositionConfig.PPG) {
                return Sequence.MLR; // Matches: M(P), L(P), R(G) -> PPG
            }
        }

        // Fallback (Should typically not be reached if inputs are valid)
        return Sequence.LMR;
    }

    /**
     * Calculates the sequence required to transform the Source (robot config)
     * into the Target (obelisk motif).
     */
    public Sequence solveSequence(PositionConfig motif, int index, boolean isBlueSide) {
        // 1. Determine Robot Configuration based on Index and Side
        PositionConfig current = getConfigForIndex(index, isBlueSide);

        // 2. Map the transformation
        return getMappedSequenceEnum(motif, current);
    }

    /**
     * Maps a specific field index (0-4) to its physical ball configuration.
     */
    public PositionConfig getConfigForIndex(int index, boolean isBlueSide) {
        PositionConfig blueConfig;

        switch (index) {
            case 0: // Preload
            case 3:
                blueConfig = PositionConfig.PPG;
                break;
            case 1: // Closest
                blueConfig = PositionConfig.GPP;
                break;
            case 2: // Middle
            case 4: // Corner
                blueConfig = PositionConfig.PGP;
                break;
            default:
                blueConfig = PositionConfig.GPP;
                break;
        }

        if (isBlueSide) {
            return blueConfig;
        } else {
            return getMirroredConfig(blueConfig);
        }
    }

    /**
     * Helper to mirror configuration for Alliance color logic if needed.
     * Mirrors indices: 0 <-> 2.
     * GPP -> PPG
     * PPG -> GPP
     * PGP -> PGP
     */
    public static PositionConfig getMirroredConfig(PositionConfig config) {
        if (config == PositionConfig.GPP) return PositionConfig.PPG;
        if (config == PositionConfig.PPG) return PositionConfig.GPP;
        return PositionConfig.PGP;
    }
}
