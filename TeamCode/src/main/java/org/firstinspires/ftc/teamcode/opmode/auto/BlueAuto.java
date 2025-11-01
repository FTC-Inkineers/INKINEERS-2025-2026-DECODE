package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Auto")
public class BlueAuto extends MainAutonomous {
    @Override
    protected boolean isBlueSide() {
        return true;
    }
}
