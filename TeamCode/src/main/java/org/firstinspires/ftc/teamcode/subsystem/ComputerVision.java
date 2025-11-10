package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class ComputerVision {
    // Alliance-specific AprilTag IDs
    private static final int BLUE_BACKDROP_ID = 20;
    private static final int RED_BACKDROP_ID = 21;

    private final Limelight3A limelight;
    private LLResult latestResult;
    private final int targetTagId;

    public ComputerVision(HardwareMap hardwareMap, boolean isBlueSide) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // April Tag Pipeline
        limelight.pipelineSwitch(1);

        // Set the target ID based on the alliance color passed in
        this.targetTagId = isBlueSide ? BLUE_BACKDROP_ID : RED_BACKDROP_ID;
    }

    public void update() {
        latestResult = limelight.getLatestResult();
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
