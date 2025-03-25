package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral.CoralSubsystem;
import frc.robot.subsystems.Other.MotorDirection;

public class CoralDispCommand extends Command {
    /*
    Manually moves the Coral dispensers using "While True" on a button
    */
    CoralSubsystem subsystem;
    MotorDirection direction;
    double endTime; //Seconds
    double currTime;
    double runTime;

    public CoralDispCommand(CoralSubsystem system, MotorDirection dir, double runTime /* in seconds */)
    {
        subsystem = system;
        direction = dir;
        this.runTime = runTime;
    }

    @Override
    public void initialize(){
        currTime = Timer.getTimestamp();
        endTime = currTime + runTime;
    }

    @Override
    public void execute()
    {
        subsystem.setDispenser(direction);
    }

    @Override
    public void end(boolean isCancelled)
    {
        subsystem.setDispenser(MotorDirection.STOP);
    }

    @Override
    public boolean isFinished()
    {
        return Timer.getTimestamp() >= endTime;
    }
}
