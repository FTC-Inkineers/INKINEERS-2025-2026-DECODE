package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class ComputerVision {
    private final Limelight3A limelight;
    private LLResult latestResult;

    public ComputerVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // April Tag Pipeline
        limelight.pipelineSwitch(1);
    }

    public void update() {
        latestResult = limelight.getLatestResult();
    }

    public LLResult getLLResult() {
        return latestResult;
    }

    /**
     * Finds and returns the largest (closest) AprilTag detected in the latest result.
     * @return The FiducialResult of the largest tag, or null if no tags are detected.
     */
    public LLResultTypes.FiducialResult getLargestTag() {
        if (latestResult == null || latestResult.getFiducialResults().isEmpty()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
        LLResultTypes.FiducialResult largestTag = null;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (largestTag == null || fiducial.getTargetArea() > largestTag.getTargetArea()) {
                largestTag = fiducial;
            }
        }
        return largestTag;
    }

    public Pose3D getLargestTagPose() {
        LLResultTypes.FiducialResult largestTag = getLargestTag();
        if (largestTag != null) {
            // Use Camera Pose Targe tSpace
            return largestTag.getCameraPoseTargetSpace();
        }
        return null;
    }

    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addLine("\\\\ LIMELIGHT //");
        LLResultTypes.FiducialResult largestTag = getLargestTag();

        if (largestTag != null) {
            telemetry.addData("Tag Found", "ID: %d, Area: %.2f", largestTag.getFiducialId(), largestTag.getTargetArea());
            Pose3D tagPose = largestTag.getCameraPoseTargetSpace();
            telemetry.addData("Pose (X, Y, Z)", "%.2f, %.2f, %.2f",
                    tagPose.getPosition().x, tagPose.getPosition().y, tagPose.getPosition().z);
        } else {
            telemetry.addData("Tag Found", "None");
        }
    }
}
