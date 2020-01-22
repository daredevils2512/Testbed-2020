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
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.controlboard.ControlBoard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PIDdrivetrain;
import frc.robot.vision.Limelight.Pipeline;
import frc.robot.commands.*;

// import frc.robot.commands.Commands;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final ControlBoard m_controlBoard = new ControlBoard();
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final Command m_autoCommand;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(new RunCommand(() -> m_drivetrain.arcadeDrive(m_controlBoard.xbox.getLeftStickY(), m_controlBoard.xbox.getRightStickX()), m_drivetrain));

    configureButtonBindings();

    m_autoCommand = new RunCommand(() -> { });
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_controlBoard.xbox.aButton.whenPressed(new InstantCommand(() -> m_drivetrain.setLowGear(true), m_drivetrain)).whenReleased(new InstantCommand(() -> m_drivetrain.setLowGear(false), m_drivetrain));
    m_controlBoard.xbox.xButton.whileHeld(Commands.motionMagic(m_drivetrain, 12));
    m_controlBoard.xbox.rightBumper.whileHeld(new FollowBall(m_drivetrain, Pipeline.PowerCells));
    m_controlBoard.xbox.leftBumper.whileHeld(new FollowBall(m_drivetrain, Pipeline.PowerCellsLimelight));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
