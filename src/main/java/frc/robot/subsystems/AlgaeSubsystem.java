package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeSubsystem extends SubsystemBase {
    /*
    Move Algae angle up/down (uses target pos, uses presets)
    Push in or out Algae
     */
    SparkMax algaeAngle = new SparkMax(Constants.CANIds.algaeAngle, SparkLowLevel.MotorType.kBrushless); //TODO CONFIRM MOTOR TYPE
    SparkMax algaePush = new SparkMax(Constants.CANIds.algaePusher, SparkLowLevel.MotorType.kBrushless); //TODO CONFIRM MOTOR TYPE

    final RelativeEncoder angleEncoder;
    double angleTargetPos;

    PIDController pid = new PIDController(0.03, 0, 0); //TODO SET P VALUE CORRECTLY

    double maxPosition = 300; //TODO SET THIS VALUE CORRECTLY
    double minPosition = 0;   //TODO SET THIS VALUE CORRECTLY

    public AlgaeSubsystem(){
        angleEncoder = algaeAngle.getEncoder();
        angleEncoder.setPosition(0);
    }

    public void setPushAlgae(MotorDirection dir) {
        switch (dir) {
            case FORWARD -> algaePush.set(Constants.SpeedConstants.ALGAE_MOVE_SPEED);
            case STOP -> algaePush.set(0);
            case REVERSE -> algaePush.set(-Constants.SpeedConstants.ALGAE_MOVE_SPEED);
        }
    }
    public void setAngleAlgae(MotorDirection dir) {
        stopMoveToPos();
        switch (dir) {
            case FORWARD -> algaeAngle.set(Constants.SpeedConstants.ALGAE_ANGLE_SPEED);
            case STOP -> algaeAngle.set(0);
            case REVERSE -> algaeAngle.set(-Constants.SpeedConstants.ALGAE_ANGLE_SPEED);
        }
    }

    public void updateAngleTarget(double change){
        angleTargetPos = MathUtil.clamp(angleTargetPos + (change * 1.0), minPosition, maxPosition);
    }

    public void tiltToPreset(AlgaePresets presets){
        angleTargetPos = presets.position;
    }

    public void stopMoveToPos(){
        angleTargetPos = angleEncoder.getPosition();
    }

    public void doPositionControl(){

    }

    @Override
    public void periodic()
    {
        double outputPower = pid.calculate(angleEncoder.getPosition(), angleTargetPos) * Constants.SpeedConstants.ALGAE_ANGLE_SPEED_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -1, 1);

        SmartDashboard.putNumber("Algae Position", angleEncoder.getPosition());
        SmartDashboard.putNumber("Algae Target Position", angleTargetPos);
        SmartDashboard.putNumber("Algae Power", outputPower);

        algaeAngle.set(outputPower);
    }

}
