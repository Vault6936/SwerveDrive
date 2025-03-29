package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Other.MotorDirection;

    public class ResetGyro extends Command {
        DriveSubsystem driveSubsystem;
        public ResetGyro(DriveSubsystem driveSubsystem){
            this.driveSubsystem = driveSubsystem;
        }

        @Override
        public void end(boolean isCancelled)
        {
            driveSubsystem.resetGyro();
        }
        @Override
        public boolean isFinished(){
            return true;
        }
}
