package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSystem;
import frc.robot.subsystems.DriveSubsystem;

public class AprilAlign extends Command {
    DriveSubsystem driveSubsystem;
    PIDController pid_hoz = new PIDController(0.5,0,0);
    PIDController pid_rot = new PIDController(0.5,0,0);


    public AprilAlign(DriveSubsystem driveSubsystem){
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    //     ☆*: .｡. o(≧▽≦)o .｡.:*☆

    @Override
    public void execute() {
        double tx = CameraSystem.tx;
        double ry = CameraSystem.ry;
        double pid_move = pid_hoz.calculate(tx,0);
        double pid_turn = MathUtil.clamp(pid_rot.calculate(ry,0), -0.5, 0.5);

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ry", ry);
        SmartDashboard.putNumber("pid_move", pid_move);
        SmartDashboard.putNumber("pid_turn", pid_turn);
        driveSubsystem.drive(-pid_move,0 ,pid_turn);
//        if (tx < -2){
//            driveSubsystem.drive(-Constants.Swerve.SPEED_OF_APRILALIGN,0.0,0.0);
//        } else if (tx > 2) {
//            driveSubsystem.drive(Constants.Swerve.SPEED_OF_APRILALIGN,0.0,0.0);
//        } else{
//            driveSubsystem.drive(0,0,0);
//        }
    }

    @Override
    public void end(boolean isCancelled)
    {
        driveSubsystem.drive(0,0,0);
    }


    @Override
    public boolean isFinished()
    {
        return CameraSystem.id == 0;
    }

}
