package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveModuleIO.ModuleData;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final SwerveModuleIO[] modules = new SwerveModuleIO[4];
  private final ModuleData[] moduleData = new ModuleData[4];

  public SwerveSubsystem()
  {
    if(Constants.robot.isSim)
    {
      for(int i=0;i<4;i++)
      {
        modules[i] = new SwerveModuleSim();
        moduleData[i] = new ModuleData();
      }
    }
  }

  public void setModuleStates(SwerveModuleState[] states)
  {
    for(int i=0;i<4;i++)
    {
      modules[i].setDesiredState(states[i]);
    }
  }

  @Override 
  public void periodic()
  {
    for(int i=0;i<4;i++)
    {
      modules[i].updateData(moduleData[i]);
    }
  }
  
}
