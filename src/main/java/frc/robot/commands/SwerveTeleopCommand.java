// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import java.util.function.DoubleSupplier;

import javax.sql.XADataSource;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SwerveTeleopCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DoubleSupplier leftX;
  private final DoubleSupplier leftY;
  private final DoubleSupplier rightX;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveTeleopCommand(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
    this.leftX = leftX;
    this.leftY = leftY;
    this.rightX = rightX;
    addRequirements(Robot.swerve);
  }

  private final SlewRateLimiter driveLimiter = new SlewRateLimiter(Constants.Swerve.maxDriveAccelerationMPSS);
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(Constants.Swerve.maxRotationAccelerationRadPSS);

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double xSpeed = leftX.getAsDouble();
    double ySpeed = -leftY.getAsDouble();
    double turningSpeed = rightX.getAsDouble();
    //dead band is because controller drift: not touching anything may equal 0.003 read
    xSpeed = xSpeed > Constants.Swerve.controllerDeadBand ? xSpeed : 0;
    ySpeed = ySpeed > Constants.Swerve.controllerDeadBand ? ySpeed : 0;
    turningSpeed = turningSpeed > Constants.Swerve.controllerDeadBand ? turningSpeed : 0;

    xSpeed = driveLimiter.calculate(xSpeed * Constants.Swerve.maxSpeedMPS);
    ySpeed = driveLimiter.calculate(ySpeed * Constants.Swerve.maxSpeedMPS);
    turningSpeed = driveLimiter.calculate(turningSpeed * Constants.Swerve.maxRotationSpeedRadPS);

    //chassis speed
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,turningSpeed, Robot.swerve.getRotation2d());

    SwerveModuleState[] states = Constants.Swerve.driveKinematics.toSwerveModuleStates(chassisSpeeds);
    Robot.swerve.setModuleStates(states);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}