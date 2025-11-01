package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Auto")
public class RedAuto extends MainAutonomous {
    @Override
    protected boolean isBlueSide() {
        return false;
    }
}
