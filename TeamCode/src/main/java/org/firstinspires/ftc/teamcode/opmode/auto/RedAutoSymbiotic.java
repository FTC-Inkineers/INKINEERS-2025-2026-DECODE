package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Auto Symbiotic", group = "Far")
public class RedAutoSymbiotic extends MainFarAutonomous {
    @Override
    protected boolean isBlueSide() {
        return false;
    }

    @Override
    protected Species getVariant() {
        return Species.SYMBIOTIC;
    }
}
