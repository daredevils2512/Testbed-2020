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
import frc.robot.subsystems.PIDDrivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.utils.HexagonPosition;
import frc.robot.vision.Limelight;
import frc.robot.vision.PiTable;
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
  private final PIDDrivetrain m_pidDrivetrain = new PIDDrivetrain();


  // @SuppressWarnings("unused")
  // private final PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  
  public final Limelight m_powerCellLimelight = new Limelight(Pipeline.PowerCellsLimelight);
  public final Limelight m_hexagonLimelight = new Limelight(Pipeline.Hexagon2d);
  private final PiTable m_piTable = new PiTable();
  
  private final Turret m_turret = new Turret();
  
  public final HexagonPosition m_hexagonPosition = new HexagonPosition(m_pidDrivetrain, m_turret, m_hexagonLimelight);

  private final Command m_autoCommand;  


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_pidDrivetrain.setDefaultCommand(new PIDDrive(m_pidDrivetrain, m_controlBoard.xbox::getLeftStickY, m_controlBoard.xbox::getRightStickX));
    m_turret.setDefaultCommand(Commands.runTurret(m_turret, m_controlBoard.extreme::getStickX));
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
    m_controlBoard.xbox.aButton.whenPressed(Commands.pidDrive(m_pidDrivetrain, 1.0, 0.0));

    // Turn to 0 degrees
    m_controlBoard.xbox.xButton.whenPressed(Commands.turnToAngle(m_pidDrivetrain, 0));
    // m_controlBoard.extreme.trigger.whileHeld(Commands.runTurretPID(m_turret, 0.0)); //was mainly for testing
    m_controlBoard.xbox.rightBumper.whileHeld(new DriveToPosition(m_pidDrivetrain, m_piTable.getClosestTarget()[2], m_piTable.getClosestTarget()[1]));

    m_controlBoard.xbox.yButton.whenPressed(Commands.resetTurret(m_turret));
    m_controlBoard.extreme.trigger.toggleWhenPressed(Commands.findTarget(m_turret, m_hexagonLimelight, 1)); //will eventually be separate limelight mounted to shooter
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
