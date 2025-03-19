package frc.robot.subsystems;

public enum CoralPresets {
    CENTER_POSITION((CoralSubsystem.minPosition + CoralSubsystem.maxPosition) / 2.);

    public final double position;
    CoralPresets(double pos)
    {
        this.position = pos;
    }
}
