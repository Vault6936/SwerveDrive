package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.DistanceSensor;

public class PerpendicularSensorSystem extends SubsystemBase {

    DistanceSensor leftDis;
    DistanceSensor rightDis;

    PIDController pid = new PIDController(10,0,0);


    public PerpendicularSensorSystem()
    {
        pid.setIntegratorRange(-1,1);
        pid.setTolerance(0.05,0.01);

        pid.setSetpoint(0);
    }

    public Boolean Complete()
    {
        return pid.atSetpoint();
    }

    public void Reset()
    {
        pid.reset();
    }

    public Double Calculate()
    {
        double currentLeftDis = leftDis.getDistance();
        double currentRightDis = rightDis.getDistance();

        double measurement = currentLeftDis - currentRightDis;


        return pid.calculate(measurement);
    }



}
