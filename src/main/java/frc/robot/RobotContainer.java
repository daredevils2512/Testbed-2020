/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.PIDdrivetrain;
import frc.robot.vision.Limelight;
import frc.robot.vision.Limelight.Pipeline;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ControlBoard m_controlBoard = new ControlBoard();
  private final PIDdrivetrain m_PIDdrivetrain = new PIDdrivetrain();

  private final PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  
  private final Limelight m_limelight = new Limelight();
  
  private final Command m_autoCommand;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_PIDdrivetrain.setDefaultCommand(new PIDDrive(m_PIDdrivetrain, m_controlBoard.xbox::getLeftStickY, m_controlBoard.xbox::getRightStickX));

    configureButtonBindings();

    m_autoCommand = null;
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_controlBoard.xbox.rightBumper.whileHeld(new FollowBall(m_PIDdrivetrain, m_limelight, Pipeline.PowerCells));
    m_controlBoard.xbox.leftBumper.whileHeld(new FollowBall(m_PIDdrivetrain, m_limelight, Pipeline.PowerCellsLimelight));
    m_controlBoard.xbox.aButton.whenPressed(Commands.pidDrive(m_PIDdrivetrain, 1.0, 0.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
