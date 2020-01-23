package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PIDdrivetrain;

public class PIDDrive extends CommandBase {
  private final PIDdrivetrain m_PIDdrivetrain;
  private double m_leftCurrentPos;
  private double m_rightCurrentPos;
  private final int kPointOffset = 12;
  private double m_forward;

  public PIDDrive(double forward) {
    m_forward = forward;
    m_PIDdrivetrain = new PIDdrivetrain();
    m_leftCurrentPos = SmartDashboard.getNumber("left inches", 0.0);
    m_rightCurrentPos = SmartDashboard.getNumber("right inches", 0.0);
  }

  @Override
  public void execute() {
    m_PIDdrivetrain.setSetPoint(m_leftCurrentPos + (m_forward * kPointOffset), m_rightCurrentPos + (m_forward * kPointOffset));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}