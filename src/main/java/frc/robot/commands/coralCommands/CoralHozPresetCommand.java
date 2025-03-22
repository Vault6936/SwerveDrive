package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralPresets;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.MotorDirection;

public class CoralHozPresetCommand extends Command {
    /*
    Moves the Coral Placer's horizontal position to a preset position defined in "CoralPresets"
    */
    CoralSubsystem subsystem;
    CoralPresets preset;
    double endTime = 0;
    public CoralHozPresetCommand(CoralSubsystem subsystem, CoralPresets preset){
        this.subsystem = subsystem;
        this.preset = preset;
    }

    @Override
    public void initialize() {
        endTime = Timer.getTimestamp() + Constants.Timeouts.coralTimeout;
        subsystem.slideToPreset(preset);
    }

    @Override
    public void execute(){}

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.stopMoveToPos();
    }

    @Override
    public boolean isFinished()
    {
        return (Math.abs(subsystem.hozEncoder.getPosition() - preset.position) < Constants.ThresholdConstants.CORAL_PRESET_THRESHOLD) || Timer.getTimestamp() > endTime;
    }
}
