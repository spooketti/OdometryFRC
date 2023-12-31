package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.intakePort, MotorType.kBrushless);

    public IntakeSubsystem() {
        //do i need to do something here
        //except maybe invert the motor 
        
    }

    public void intakeVoltage(double volts)
    {
        intakeMotor.setVoltage(volts);
    }

    public void outtakeVoltage(double volts)
    {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void periodic() //we don't need this
    {
        SmartDashboard.putNumber("IntakeVolts",intakeMotor.getBusVoltage());

    }

}
