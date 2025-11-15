package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Auto Close", group = "BlueClose")
public class BlueAutoClose extends MainCloseAutonomous {
    @Override
    protected boolean isBlueSide() {
        return true;
    }

    @Override
    protected Species getVariant() {
        return Species.SYMBIOTIC;
    }
}
