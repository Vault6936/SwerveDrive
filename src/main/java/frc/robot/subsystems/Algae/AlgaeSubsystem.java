package frc.robot.subsystems.Algae;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Other.MotorDirection;

public class AlgaeSubsystem extends SubsystemBase {
    /*
    Move Algae angle up/down (uses target pos, uses presets)
    Push in or out Algae
     */
    SparkMax algaeAngle = new SparkMax(Constants.CANIds.algaeAngle, SparkLowLevel.MotorType.kBrushed);
    SparkMax algaePush = new SparkMax(Constants.CANIds.algaePusher, SparkLowLevel.MotorType.kBrushless);

    RelativeEncoder angleEncoder;
    double angleTargetPos;


    PIDController pid = new PIDController(30, 0, 0); //TODO SET P VALUE CORRECTLY

    double maxPosition = .66;
    double minPosition = 0;

    public AlgaeSubsystem(){
        angleEncoder = algaeAngle.getEncoder();
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
        angleTargetPos =angleTargetPos + change * .0035;
        //angleTargetPos = MathUtil.clamp(angleTargetPos + (change * .1), minPosition, maxPosition);
    }

    public void tiltToPreset(AlgaePresets presets){
        angleTargetPos = presets.position;
    }

    public void stopMoveToPos(){
        angleTargetPos = getAngle();
    }

    public void doPositionControl(){
        double outputPower = pid.calculate(getAngle(), angleTargetPos) * Constants.SpeedConstants.ALGAE_ANGLE_SPEED_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -1, 1);
        //if (Constants.DebugInfo.debugAlgae)
        //{
            SmartDashboard.putNumber("Algae Power", outputPower);
        //}

        algaeAngle.set(outputPower);
    }

    public double getAngle()
    {
        return angleEncoder.getPosition();
    }

    public void setAngle(double angle){
        angleEncoder.setPosition(angle);
    }
    @Override
    public void periodic()
    {
        doPositionControl();
        //if (Constants.DebugInfo.debugAlgae) {
            SmartDashboard.putNumber("Algae Position", getAngle());
            SmartDashboard.putNumber("Algae Target Position", angleTargetPos);
        //}

    }

}
