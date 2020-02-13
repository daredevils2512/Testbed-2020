/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDDrivetrain;

public class DriveToPosition extends CommandBase {
  private PIDController m_leftController;
  private PIDController m_rightController;
  private PIDDrivetrain m_drivetrain;
  private NetworkTable m_table;

  private double k_lP;
  private double k_lI;
  private double k_lD;

  private double k_rP;
  private double k_rI;
  private double k_rD;

  private double m_move;
  private double m_turn;
  private double m_leftDistance;
  private double m_rightDistance;
  private double m_leftSpeed;
  private double m_rightSpeed;
  private double m_deltaAngle;
  private double m_radius;

  /**
   * 
   * @param drivetrain
   * @param move meters foreward
   * @param heading degrees offset
   */
  public DriveToPosition(PIDDrivetrain drivetrain, double move, double heading) {
    m_drivetrain = drivetrain;
    m_leftController = new PIDController(k_lP, k_lI, k_lD);
    m_rightController = new PIDController(k_rP, k_rI, k_rD);
    addRequirements(drivetrain);
    m_table = NetworkTableInstance.getDefault().getTable("pid drivetrain");
    m_table.getEntry("left P").setDouble(k_lP);
    m_table.getEntry("left I").setDouble(k_lI);
    m_table.getEntry("left D").setDouble(k_lD);

    m_table.getEntry("right P").setDouble(k_rP);
    m_table.getEntry("right I").setDouble(k_rI);
    m_table.getEntry("right D").setDouble(k_rD);

    m_move = move;
    m_turn = heading;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    k_lP = m_table.getEntry("left P").getDouble(0.0);
    k_lI = m_table.getEntry("left I").getDouble(0.0);
    k_lD = m_table.getEntry("left D").getDouble(0.0);
  
    k_rP = m_table.getEntry("right P").getDouble(0.0);
    k_rI = m_table.getEntry("right I").getDouble(0.0);
    k_rD = m_table.getEntry("right D").getDouble(0.0);

    if (m_turn != 0.0) {
      m_deltaAngle = Math.toRadians(m_turn * 2);
      m_radius = m_move / m_deltaAngle;
      m_rightDistance = 2 * Math.PI * (m_radius + m_drivetrain.m_trackWidth) * (m_deltaAngle / (2 * Math.PI));
      m_leftDistance = 2 * Math.PI * (m_radius - m_drivetrain.m_trackWidth) * (m_deltaAngle / (2 * Math.PI));
    } else {
      m_rightDistance = m_move;
      m_leftDistance = m_move;
    }

    m_leftController.setSetpoint(m_leftDistance + m_drivetrain.getLeftEncoderDistance());
    m_rightController.setSetpoint(m_rightDistance + m_drivetrain.getRightEncoderDistance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leftSpeed = m_leftController.calculate(m_drivetrain.getLeftEncoderDistance());
    m_rightSpeed = m_rightController.calculate(m_drivetrain.getRightEncoderDistance());
    m_drivetrain.driveMotors(new DifferentialDriveWheelSpeeds(m_leftSpeed, m_rightSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_leftController.atSetpoint() && m_rightController.atSetpoint());
  }
}
