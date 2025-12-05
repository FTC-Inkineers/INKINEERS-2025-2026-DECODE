package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

public class CannonSailClose extends CannonSail {

    private final double rpm;

    public CannonSailClose(ShooterSubsystem shooter, IntakeSubsystem intake, SequenceMapper.Sequence sequence, int index) {
        super(shooter, intake, sequence, index);

        rpm = shooter.getStationaryRPM_Close();
    }

    @Override
    protected double targetRpm() {
        return rpm;
    }
}
