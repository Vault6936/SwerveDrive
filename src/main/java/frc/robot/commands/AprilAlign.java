package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.vision.LimelightHelpers;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.DriveSubsystem;

public class AprilAlign extends Command {
    double hoz_off;
    DriveSubsystem driveSubsystem;
    PIDController pid_control = new PIDController(0.05,0,0);

    public AprilAlign(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    //     ☆*: .｡. o(≧▽≦)o .｡.:*☆

    @Override
    public void execute() {
        double tx = CameraSystem.tx;
        this.hoz_off = tx;
        double pid_calc = pid_control.calculate(tx,0);
        SmartDashboard.putNumber("PID VALUE: ",pid_calc);
        driveSubsystem.drive(pid_calc,0,0);

//        if (tx < -2){
//            driveSubsystem.drive(-Constants.Swerve.SPEED_OF_APRILALIGN,0.0,0.0);
//        } else if (tx > 2) {
//            driveSubsystem.drive(Constants.Swerve.SPEED_OF_APRILALIGN,0.0,0.0);
//        } else{
//            driveSubsystem.drive(0,0,0);
//        }
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

}
