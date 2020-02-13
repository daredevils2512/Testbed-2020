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
import frc.robot.commands.DriveToPosition;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.Extreme;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drivetrain.AtlasDrivetrain;
import frc.robot.subsystems.drivetrain.PIDDrivetrain;
import frc.robot.utils.HexagonPosition;
import frc.robot.vision.Limelight;
import frc.robot.vision.PiTable;
import frc.robot.vision.Limelight.Pipeline;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Extreme m_extreme = new Extreme(0);
  private final ControlBoard m_controlBoard = new ControlBoard();

  // @SuppressWarnings("unused")
  // private final PowerDistributionPanel m_pdp = new PowerDistributionPanel();

  public final Limelight m_powerCellLimelight = new Limelight(Pipeline.PowerCellsLimelight);
  public final Limelight m_hexagonLimelight = new Limelight(Pipeline.Hexagon2d);
  private final PiTable m_piTable = new PiTable();
  
  private final PIDDrivetrain m_pidDrivetrain =new PIDDrivetrain();
  private final AtlasDrivetrain m_atlasDrivetrain = new AtlasDrivetrain();
  private final Turret m_turret = new Turret();
  
  public final HexagonPosition m_hexagonPosition = new HexagonPosition(m_atlasDrivetrain, m_turret, m_hexagonLimelight);

  private final Command m_autoCommand;  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_atlasDrivetrain.setDefaultCommand(Commands.velocityArcadeDrive(
      m_atlasDrivetrain,
      () -> -m_extreme.getStickY(0.3),
      () -> m_extreme.getStickRotation(0.3)));
    m_turret.setDefaultCommand(Commands.runTurret(
      m_turret,
      () -> m_extreme.getPOVX()));

    configureButtonBindings();

    m_autoCommand = null;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_extreme.baseFrontLeft.whenPressed(Commands.resetPose(m_atlasDrivetrain));

    m_extreme.trigger.whileHeld(Commands.runTurretPID(m_turret, 0.0));
    // m_controlBoard.extreme.trigger.whileHeld(Commands.runTurretPID(m_turret, 0.0)); //was mainly for testing
    m_controlBoard.xbox.rightBumper.whileHeld(new DriveToPosition(m_pidDrivetrain, m_piTable.getClosestTarget()[2], m_piTable.getClosestTarget()[1]));

    m_extreme.sideButton.whenPressed(Commands.resetTurret(m_turret));
    m_extreme.trigger.toggleWhenPressed(Commands.findTarget(m_turret, m_hexagonLimelight, 1)); //will eventually be separate limelight mounted to shooter
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
