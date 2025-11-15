package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Auto Close", group = "RedClose")
public class RedAutoClose extends MainCloseAutonomous {
    @Override
    protected boolean isBlueSide() {
        return false;
    }

    @Override
    protected Species getVariant() {
        return Species.SYMBIOTIC;
    }
}
