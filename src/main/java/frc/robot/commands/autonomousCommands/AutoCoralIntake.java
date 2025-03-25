package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Other.MotorDirection;

public class AutoCoralIntake extends Command {
    CoralSubsystem coralSubsystem;
    double endTime;

    public AutoCoralIntake(CoralSubsystem coralSystem){
        coralSubsystem = coralSystem;
    }
    @Override
    public void initialize(){
        endTime = Timer.getTimestamp() + Constants.Timeouts.coralTimeout;
    }

    @Override
    public void execute()
    {
        coralSubsystem.setDispenser(MotorDirection.REVERSE);
    }

    @Override
    public void end(boolean isCancelled)
    {
        coralSubsystem.setDispenser(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished()
    {
        return Timer.getTimestamp() >= endTime || coralSubsystem.getGateBool();
    }
}
