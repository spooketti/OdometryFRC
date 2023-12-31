package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSimulation extends SubsystemBase{
    private FlywheelSim intakeSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private double motorDir;

    public IntakeSimulation() {
        //do i need to do something here
        //except maybe invert the motor 
        
    }

    public void intakeVoltage(double volts)
    {
        intakeSim.setInputVoltage(volts);
        motorDir = volts;
    }

    public void outtakeVoltage(double volts)
    {
        intakeSim.setInputVoltage(volts);
        motorDir = volts;
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("IntakeVolts",intakeSim.getCurrentDrawAmps());
        SmartDashboard.putNumber("IntakeDirection", motorDir);

    }

}
