package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
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
    SparkMax algaeAngle = new SparkMax(Constants.CANIds.algaeAngle, SparkLowLevel.MotorType.kBrushed);
    SparkMax algaePush = new SparkMax(Constants.CANIds.algaePusher, SparkLowLevel.MotorType.kBrushless);

    public final AnalogInput angleValue = new AnalogInput() {
        @Override
        public double getVoltage() {
            return 0;
        }

        @Override
        public double getPosition() {
            return 0;
        }
    };
    double angleTargetPos;


    PIDController pid = new PIDController(0.03, 0, 0); //TODO SET P VALUE CORRECTLY

    double maxPosition = 1;
    double minPosition = 0;
    double algae_hit_lift_pos = .5; //TODO This value may not be correct

    boolean isSafeToLower = true;

    public AlgaeSubsystem(AnalogInput angleValue){
        this.angleValue = angleValue;
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

    public void setSafePos()
    {
        if ((Math.abs(angleValue.getPosition() - angleTargetPos) < 10) &&
                (Math.abs(angleTargetPos) < 10 ))   //TODO SET TOLERANCE
        {
            isSafeToLower = true;
        } else {
            //isSafeToLower = false;
            tiltToPreset(AlgaePresets.DEFAULT_DOWN);
        }
    }

    public void updateAngleTarget(double change){
        angleTargetPos = MathUtil.clamp(angleTargetPos + (change * .1), minPosition, maxPosition);
    }



    public void tiltToPreset(AlgaePresets presets){
        angleTargetPos = presets.position;
    }

    public void stopMoveToPos(){
        angleTargetPos = angleValue.getPosition();
    }

    public void doPositionControl(){
        double outputPower = pid.calculate(angleValue.getPosition(), angleTargetPos) * Constants.SpeedConstants.ALGAE_ANGLE_SPEED_MAGNIFIER;
        outputPower = MathUtil.clamp(outputPower, -1, 1);
        SmartDashboard.putNumber("Algae Power", outputPower);
        algaeAngle.set(outputPower);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Algae Position", angleValue.getPosition());
        SmartDashboard.putNumber("Algae Target Position", angleTargetPos);

    }

}
