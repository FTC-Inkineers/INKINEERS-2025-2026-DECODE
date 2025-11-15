package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Auto Symbiotic", group = "Far")
public class BlueAutoSymbiotic extends MainFarAutonomous {
    @Override
    protected boolean isBlueSide() {
        return true;
    }

    @Override
    protected Species getVariant() {
        return Species.SYMBIOTIC;
    }
}
