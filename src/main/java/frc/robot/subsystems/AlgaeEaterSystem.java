package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeEaterSystem extends SubsystemBase {
    SparkMax intake = new SparkMax(Constants.CANIds.algae, SparkLowLevel.MotorType.kBrushless);

    public void setIntake(MotorDirection dir) {
        switch (dir) {
            case FORWARD -> intake.set(1);
            case STOP -> intake.set(0);
            case REVERSE -> intake.set(-1);
        }
    }
}
