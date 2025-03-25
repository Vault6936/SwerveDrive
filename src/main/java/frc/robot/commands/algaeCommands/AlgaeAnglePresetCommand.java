package frc.robot.commands.algaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae.AlgaePresets;
import frc.robot.subsystems.Algae.AlgaeSubsystem;

public class AlgaeAnglePresetCommand extends Command {
    /*
    Moves the Algae Placer's angle to a preset position defined in "AlgaePresets"
     */
    AlgaeSubsystem subsystem;
    AlgaePresets preset;

    public AlgaeAnglePresetCommand(AlgaeSubsystem subsystem, AlgaePresets preset) {
        this.subsystem = subsystem;
        this.preset = preset;
    }

    @Override
    public void initialize() {
        subsystem.tiltToPreset(preset);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean isCancelled) {
        subsystem.stopMoveToPos();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(subsystem.getAngle() - preset.position) < Constants.ThresholdConstants.ALGAE_PRESET_THRESHOLD);
    }
}
