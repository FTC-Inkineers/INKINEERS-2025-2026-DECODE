package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class VisionSubsystem {
    // Alliance-specific AprilTag IDs
    private static final int BLUE_GOAL_ID = 20;
    private static final int RED_GOAL_ID = 24;
    private static final int OBELISK_GPP_ID = 21;
    private static final int OBELISK_PGP_ID = 22;
    private static final int OBELISK_PPG_ID = 23;
    
    public enum ObeliskMotif {
        GPP,
        PGP,
        PPG,
        UNKNOWN
    }
    private ObeliskMotif motif = ObeliskMotif.UNKNOWN;

    private final Limelight3A limelight;
    private LLResult latestResult;
    private final int targetTagId;

    public VisionSubsystem(HardwareMap hardwareMap, boolean isBlueSide) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // April Tag Pipeline
        limelight.pipelineSwitch(1);

        // Set the target ID based on the alliance color passed in
        this.targetTagId = isBlueSide ? BLUE_GOAL_ID : RED_GOAL_ID;
    }

    public void update() {
        latestResult = limelight.getLatestResult();
    }

    public ObeliskMotif detectObeliskMotif() {
        if (latestResult == null || latestResult.getFiducialResults().isEmpty()) {
            return ObeliskMotif.UNKNOWN;
        }

        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            switch (fiducial.getFiducialId()) {
                case OBELISK_GPP_ID:
                    motif = ObeliskMotif.GPP;
                    return motif;
                case OBELISK_PGP_ID:
                    motif = ObeliskMotif.PGP;
                    return motif;
                case OBELISK_PPG_ID:
                    motif = ObeliskMotif.PPG;
                    return motif;
            }
        }
        return ObeliskMotif.UNKNOWN;
    }

    @SuppressWarnings("unused")
    public ObeliskMotif getObeliskMotif() {
        return motif;
    }

    public boolean isTargetVisible() {
        return getTargetTag() != null;
    }

    /**
     * Finds and returns the specified alliance AprilTag from the latest result.
     * @return The FiducialResult of the target tag, or null if it's not detected.
     */
    public LLResultTypes.FiducialResult getTargetTag() {
        if (latestResult == null || latestResult.getFiducialResults().isEmpty()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == targetTagId) {
                return fiducial; // Return the first match with the correct ID
            }
        }
        return null;
    }

    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addLine("\\\\ LIMELIGHT //");
        telemetry.addData("Targeting Tag ID", targetTagId);
        LLResultTypes.FiducialResult targetTag = getTargetTag();

        if (targetTag != null) {
            telemetry.addData("Tag Found", "ID: %d, Area: %.2f", targetTag.getFiducialId(), targetTag.getTargetArea());
            telemetry.addData("Tag X degrees", targetTag.getTargetXDegrees());
            telemetry.addData("Tag Y degrees", targetTag.getTargetYDegrees());
            Pose3D tagPose = targetTag.getCameraPoseTargetSpace();
            telemetry.addData("Pose (X, Y, Z)", "%.2f, %.2f, %.2f",
                    tagPose.getPosition().x, tagPose.getPosition().y, tagPose.getPosition().z);
        } else {
            telemetry.addData("Tag Found", "None");
        }
    }
}
