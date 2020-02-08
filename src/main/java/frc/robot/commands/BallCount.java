package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PhotoEyeSubsystem;

public class BallCount extends CommandBase {
  private static PhotoEyeSubsystem m_photoEyeSubsystem;

  public BallCount(PhotoEyeSubsystem photoEyeSubsystem) {
    m_photoEyeSubsystem = photoEyeSubsystem;
    addRequirements(m_photoEyeSubsystem);
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  

}