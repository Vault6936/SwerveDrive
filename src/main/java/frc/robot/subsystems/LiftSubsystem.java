package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class LiftSubsystem extends SubsystemBase {
    /*

     */
    TalonFX extend =  new TalonFX(Constants.CANIds.lift);
    PIDController pid = new PIDController(0.25, 0, 0);
    double currentTargetPos = 0;
    static double min_position = -380.0;
    static double max_position = 0.0;
    DoubleSupplier encoder_value;
    DoubleConsumer setSpeedMultiplier;

    public LiftSubsystem(DoubleConsumer setSpeedMultiplier)
    {
        encoder_value = () -> extend.getPosition().getValueAsDouble();
        currentTargetPos = encoder_value.getAsDouble();
    }

    public double getDriveSpeedMultiplier(){
        return .8 / (1 + 2 * Math.pow(Math.E,.02 * (encoder_value.getAsDouble() - 2./3. * max_position))) + .2;
    }

    public void updatePos(double change)
    {
        currentTargetPos = MathUtil.clamp(currentTargetPos + (change * 3.2), min_position, max_position);
    }

    public void goPreset(LiftPresets preset){
        currentTargetPos = preset.position;
    }

    public void stopMoveToPos()
    {
        currentTargetPos = encoder_value.getAsDouble();
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
    public void doPositionControl(){
        double outputPower = pid.calculate(encoder_value.getAsDouble(), currentTargetPos) * Constants.SpeedConstants.LIFT_SPEED_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -0.3, 0.3);
        SmartDashboard.putNumber("LiftPower", outputPower);
        extend.set(outputPower);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("LiftPosition", encoder_value.getAsDouble());
        SmartDashboard.putNumber("LiftTargetPosition", currentTargetPos);
    }
}
