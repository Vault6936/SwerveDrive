package frc.robot.commands.algaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae.AlgaeSubsystem;


import java.util.function.DoubleSupplier;

public class AlgaeAnglePidControl extends Command {
    /*
    */
    AlgaeSubsystem subsystem;
    DoubleSupplier angle;

    public AlgaeAnglePidControl(AlgaeSubsystem subsystem, DoubleSupplier angle) {
        this.subsystem = subsystem;
        this.angle = angle;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        subsystem.updateAngleTarget(angle.getAsDouble());
        subsystem.doPositionControl();
    }

    @Override
    public void end(boolean isCancelled) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

