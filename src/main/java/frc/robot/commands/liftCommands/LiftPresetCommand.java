package frc.robot.commands.liftCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift.LiftPresets;
import frc.robot.subsystems.Lift.LiftSubsystem;

public class LiftPresetCommand  extends Command {

    private final LiftSubsystem liftSubsystem;
    LiftPresets target_preset;

    public LiftPresetCommand(LiftSubsystem liftSubsystem, LiftPresets preset)
    {
        this.liftSubsystem = liftSubsystem;
        target_preset = preset;
    }

    @Override
    public void initialize() {
        liftSubsystem.goPreset(target_preset);
    }

    @Override
    public void end(boolean cancelled)
    {
        liftSubsystem.stopMoveToPos();
    }

    @Override
    public boolean isFinished()
    {
        return Math.abs(liftSubsystem.getCurrentPosition() - target_preset.position) < .1;
    }
}
