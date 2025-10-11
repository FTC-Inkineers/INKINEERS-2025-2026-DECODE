package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ComputerVision {
    Limelight3A limelight;

    public ComputerVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }


}
