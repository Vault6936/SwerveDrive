package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSystem extends SubsystemBase {
    SparkMax extend = new SparkMax(Constants.CANIds.lift, SparkLowLevel.MotorType.kBrushless);
    RelativeEncoder encoder;
    PIDController pid = new PIDController(0.25, 0, 0);
    double currentTargetPos = 0;
    double min_position = -380.0;
    double max_position = 0.0;


    public LiftSystem()
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

    public void stop()
    {
        currentTargetPos = encoder.getPosition();
    }

    public void setExtend(MotorDirection dir) {
        switch (dir) {
            case FORWARD :
            {
                extend.set(0.4);  //do we need to change the speed?
            }
            case STOP :
            {
                extend.set(0);
                currentTargetPos = encoder.getPosition();
            }
            case REVERSE :
            {
                extend.set(-0.4);
            }
        }
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("LiftPosition", encoder.getPosition());
        SmartDashboard.putNumber("LiftTargetPosition", currentTargetPos);
        double outputPower = pid.calculate(encoder.getPosition(), currentTargetPos);
        SmartDashboard.putNumber("LiftPower", outputPower);
        outputPower = MathUtil.clamp(outputPower, -1, 1);

        extend.set(outputPower);
    }
}
