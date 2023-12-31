// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class OneOneAuto extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PIDController controller = new PIDController(1, 0, 0);
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Swerve.maxDriveAccelerationMPSS);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Swerve.maxDriveAccelerationMPSS);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OneOneAuto() {
    addRequirements(Robot.swerve);
  }

 
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double xSpeed = controller.calculate(Robot.swerve.getPose().getX(), 1);
    double ySpeed = controller.calculate(Robot.swerve.getPose().getY(), 1);
    xSpeed = xLimiter.calculate(xSpeed) * Constants.Swerve.maxDriveSpeedMPS;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.Swerve.maxDriveSpeedMPS;
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,0, Robot.swerve.getRotation2d());
    //SmartDashboard.putNumber("autoX", Robot.swe);
    //SmartDashboard.putNumber("autoY", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumberArray("2dpose", new double[] {xSpeed, ySpeed});
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
