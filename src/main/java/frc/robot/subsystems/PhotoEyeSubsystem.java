package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotoEyeSubsystem extends SubsystemBase {
  private final int m_photoeye1ID = -1;
  private final int m_photoeye2ID = -1;
  private DigitalInput m_photoeye1;
  private DigitalInput m_photoeye2;
  
  //these are for the ball counter
  private int ballCount;
  private boolean ballIn;
  private boolean ballOut;
  private boolean invalidBallCount;

  /**
   * Creates a new PowerCellManager.
   */
  public PhotoEyeSubsystem() {
    m_photoeye1 = new DigitalInput(m_photoeye1ID);
    m_photoeye2 = new DigitalInput(m_photoeye2ID);
    
    ballCount = 0;
    ballIn = false;
    ballOut = false;
    invalidBallCount = false;
  }
  
  public boolean getInBall() {
    return !m_photoeye1.get();
  }

  public boolean getOutBall() {
    return !m_photoeye2.get();
  }

  public void setBallsInMag(int set) {
    ballCount = set;
  }

  public void resetBallCount() {
    setBallsInMag(0);
  }

  public boolean getInvalidBallCount() {
    return invalidBallCount;
  }
  
  //might be temporary
  public int countBall() {
    if (getInBall()) {
      ballIn = true;
    } else if (!getInBall() && ballIn) {
      ballIn = false;
      ballCount += 1;
    } else if (getOutBall()) {
     ballOut = true;
    } else if (!getOutBall() && ballOut) {
      ballOut = false;
      ballCount -= 1;
    }
    if (ballCount < 0 || ballCount > 3) {
      System.out.println("INVALID BALL COUNT");
      invalidBallCount = true;
    } else {
      invalidBallCount = false;
    }
    return ballCount;
  }

  @Override
  public void periodic() {
    countBall();
    SmartDashboard.putNumber("balls in mag", countBall());
    SmartDashboard.putBoolean("invalid ball count", getInvalidBallCount());
  }
}
