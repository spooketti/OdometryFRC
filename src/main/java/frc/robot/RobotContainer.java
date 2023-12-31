// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.OneOneAuto;
import frc.robot.commands.SwerveTeleopCommand;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController pilot =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DataLogManager.start(".wpilog");
    DataLogManager.logNetworkTables(true);
    
        DriverStation.startDataLog(DataLogManager.getLog(), true);
  }

  private void configureBindings() 
  {
    Robot.swerve.setDefaultCommand(new SwerveTeleopCommand(
      () -> -pilot.getLeftY(),
      () -> -pilot.getLeftX(),
      () -> -pilot.getRightX()
    ));
    pilot.leftTrigger().onTrue(Commands.runOnce(() -> Robot.intakeSim.intakeVoltage(Constants.Intake.IntakeVoltage)));
    pilot.leftTrigger().onFalse(Commands.runOnce(() -> Robot.intakeSim.intakeVoltage(0)));
    pilot.rightTrigger().onTrue(Commands.runOnce(() -> Robot.intakeSim.outtakeVoltage(Constants.Intake.OuttakeVoltage)));
    pilot.rightTrigger().onFalse(Commands.runOnce(() -> Robot.intakeSim.outtakeVoltage(0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PrintCommand("autoless so sad");//OneOneAuto();
  }
}
