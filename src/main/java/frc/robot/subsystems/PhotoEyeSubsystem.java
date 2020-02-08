package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.PhotoEye;

public class PhotoEyeSubsystem extends SubsystemBase {
  private final int m_photoeye1ID = 0;
  private final int m_photoeye2ID = 1;
  private final PhotoEye m_photoeye1;
  private final PhotoEye m_photoeye2;
  
  //these are for the ball counter
  private int ballCount;
  private boolean ballIn;
  private boolean ballOut;
  private boolean invalidBallCount;

  /**
   * Creates a new PowerCellManager.
   */
  public PhotoEyeSubsystem() {
    m_photoeye1 = new PhotoEye(m_photoeye1ID);
    m_photoeye2 = new PhotoEye(m_photoeye2ID);
    
    ballCount = 0;
    ballIn = false;
    ballOut = false;
    invalidBallCount = false;
  }
  
  public boolean getInBall() {
    return !m_photoeye1.getDetected();
  }

  public boolean getOutBall() {
    return !m_photoeye2.getDetected();
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
      invalidBallCount = true;
    } else {
      invalidBallCount = false;
    }
    return ballCount;
  }

  public void updateBall() {
    countBall();
    SmartDashboard.putNumber("balls in magazine", countBall());
    SmartDashboard.putBoolean("invalid ball count", getInvalidBallCount());
    if (getInvalidBallCount()) {
      System.out.println("INVALID BALL COUNT");
    }
  }

  @Override
  public void periodic() {
  }
}