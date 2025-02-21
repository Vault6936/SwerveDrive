package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
    /*

     */
    SparkMax extend = new SparkMax(Constants.CANIds.lift, SparkLowLevel.MotorType.kBrushless);
    RelativeEncoder encoder;
    PIDController pid = new PIDController(0.25, 0, 0);
    double currentTargetPos = 0;
    double min_position = -380.0;
    double max_position = 0.0;

    public LiftSubsystem()
    {
        encoder = extend.getEncoder();
        encoder.setPosition(0);
    }

    public void updatePos(double change)
    {
        currentTargetPos = MathUtil.clamp(currentTargetPos + (change * 3.2),min_position,max_position);

    }

    public void goPreset(LiftPresets preset){
        currentTargetPos = preset.position;
    }

    public void stopMoveToPos()
    {
        currentTargetPos = encoder.getPosition();
    }

    public void setExtend(MotorDirection dir) {
        stopMoveToPos();
        switch (dir) {
            case FORWARD :
            {
                extend.set(Constants.SpeedConstants.LIFT_SPEED);
            }
            case STOP :
            {
                extend.set(0);
            }
            case REVERSE :
            {
                extend.set(-Constants.SpeedConstants.LIFT_SPEED);
            }
        }
    }

    @Override
    public void periodic()
    {
        double outputPower = pid.calculate(encoder.getPosition(), currentTargetPos) * Constants.SpeedConstants.LIFT_SPEED_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -1, 1);

        SmartDashboard.putNumber("LiftPosition", encoder.getPosition());
        SmartDashboard.putNumber("LiftTargetPosition", currentTargetPos);
        SmartDashboard.putNumber("LiftPower", outputPower);

        extend.set(outputPower);
    }
}
