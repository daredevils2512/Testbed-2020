package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PIDdrivetrain;

public class PIDDrive extends CommandBase {
  private final PIDdrivetrain m_PIDdrivetrain;
  private double m_leftCurrentPos;
  private double m_rightCurrentPos;
  private double m_forward;

  public PIDDrive(PIDdrivetrain PIDdrivetrain, double forward) {
    m_forward = forward;
    m_PIDdrivetrain = PIDdrivetrain;
    m_leftCurrentPos = SmartDashboard.getNumber("left inches", 0.0);
    m_rightCurrentPos = SmartDashboard.getNumber("right inches", 0.0);
  }

  @Override
  public void execute() {
    System.out.println("dufhkncsajmfdkxaxfskjdahfkdsaljomfciahlmfxkajhfcomiahef,xljahfcoimauhlfexakuhfncauesgfiurehfcakufehal");
    m_PIDdrivetrain.setSetPoint(m_forward + m_leftCurrentPos, m_forward + m_rightCurrentPos);
  }

  

  @Override
  public boolean isFinished() {
    return false;
  }
}