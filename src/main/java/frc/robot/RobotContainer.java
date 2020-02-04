/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Commands;
import frc.robot.controlboard.Extreme;
import frc.robot.subsystems.drivetrain.AleaDrivetrain;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Extreme m_extreme = new Extreme(0);
  @SuppressWarnings("unused")
  private final PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  
  private final AleaDrivetrain m_aleaDrivetrain = new AleaDrivetrain();

  private final Command m_autoCommand;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_aleaDrivetrain.setDefaultCommand(Commands.simpleArcadeDrive(
      m_aleaDrivetrain, 
      () -> getMove(), 
      () -> getTurn()));

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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  public double getMove() {
    return -m_extreme.getStickY(0.3);
  }

  public double getTurn() {
    return m_extreme.getStickRotation(0.3);
  }
}
