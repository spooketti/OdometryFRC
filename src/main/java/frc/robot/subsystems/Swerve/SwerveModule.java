package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule implements SwerveModuleIO {
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private CANCoder absoluteEncoder;
    private RelativeEncoder driveEncoder;

    private PIDController drivePID = new PIDController(1.5, 0, 0);
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0, 2.93);
    private PIDController turnPID = new PIDController(1, 0, 0);

    public SwerveModule(int deviceID) {

        absoluteEncoder = new CANCoder(Constants.Swerve.absoluteEncoderPorts[deviceID]);
        driveMotor = new CANSparkMax(Constants.Swerve.drivePorts[deviceID], MotorType.kBrushless); // will error use an arary in constants devID as the index
        turnMotor = new CANSparkMax(Constants.Swerve.turnPorts[deviceID], MotorType.kBrushless);
        
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        //RelativeEncoder driveEncoder = driveMotor.getEncoder();
        //driveEncoder.setPositionConversionFactor(Constants.Swerve.kDriveMotorGearRatio);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.Swerve.kDriveMotorGearRatio);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.kDriveMotorGearRatio/60);
        turnPID.enableContinuousInput(0, 2 * Math.PI);
    }

    private SwerveModuleState theoreticalState = new SwerveModuleState();
    //private double drivePositionM = 0.0;
    private double driveAppliedVolts = 0.0;

    private double turnPositionRad = 0.0;
    private double turnAppliedVolts = 0.0;


    // check swervemoduleio for the comments
    @Override
    public void updateData(ModuleData data) {

        data.drivePositionM = driveEncoder.getPosition();
        data.driveVelocityMPerSec = driveEncoder.getVelocity() * (Constants.Swerve.wheelDiameterM / 2);
        data.driveCurrentAmps = driveMotor.getBusVoltage();//Math.abs(driveMotor.getOutputCurrent());
        data.driveAppliedVolts = driveAppliedVolts;
        data.driveTempCelcius = driveMotor.getMotorTemperature();


        turnPositionRad = absoluteEncoder.getAbsolutePosition() / 180 * Math.PI;
        data.turnPositionRad = turnPositionRad;
        data.turnVelocityRadPerSec = absoluteEncoder.getVelocity() / 180 * Math.PI;
        data.turnCurrentAmps = turnMotor.getBusVoltage();//Math.abs(turnMotor.getOutputCurrent());
        data.turnTempCelcius = turnMotor.getMotorTemperature();
        data.turnAppliedVolts = turnAppliedVolts;


        data.theoreticalState = theoreticalState;
    }

    private SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity() * Constants.Swerve.wheelDiameterM / 2,
                new Rotation2d(turnPositionRad));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {

        state = SwerveModuleState.optimize(state,getState().angle);
        theoreticalState = state;

        if(Math.abs(state.speedMetersPerSecond) < 0.01)
        {
            state.speedMetersPerSecond = 0;
        }
        double driveVolts = driveFF.calculate(state.speedMetersPerSecond) 
            + drivePID.calculate(driveEncoder.getVelocity() * Constants.Swerve.wheelDiameterM / 2,
            state.speedMetersPerSecond) ;
        double turnVolts = turnPID.calculate(turnPositionRad, state.angle.getRadians());
        setDriveVoltage(driveVolts);
        setTurnVoltage(turnVolts);

    }

    @Override
    public void stop() {
        setDriveVoltage(0);
        setTurnVoltage(0);
    }

    private void setDriveVoltage(double volts) {
        driveAppliedVolts = volts;
        driveMotor.setVoltage(driveAppliedVolts);
    }

    private void setTurnVoltage(double volts) {
        turnAppliedVolts = volts;
        turnMotor.setVoltage(turnAppliedVolts);
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
    }

    @Override
    public void setTurningBrakeMode(boolean enable) {
    }
}
