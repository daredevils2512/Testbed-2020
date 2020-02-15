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
import frc.robot.controlboard.Extreme;
import frc.robot.commands.Commands;
import frc.robot.subsystems.drivetrain.Drivetrain2020;
import frc.robot.utils.DriveType;
import frc.robot.vision.Pipeline;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Extreme m_extreme = new Extreme(0);

  // @SuppressWarnings("unused")
  // private final PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  
  private final Drivetrain2020 m_drivetrain = new Drivetrain2020();
  private final Turret m_turret = new Turret();
  
  private final Command m_autoCommand;  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrain.setDefaultCommand(Commands.velocityArcadeDrive(
      m_drivetrain,
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
    m_extreme.baseFrontLeft.whenPressed(Commands.resetPose(m_drivetrain));

    m_extreme.baseFrontRight.whenPressed(Commands.resetTurret(m_turret));
    m_extreme.baseMiddleLeft.whileHeld(Commands.runTurretPID(m_turret, 0.0));
    m_extreme.trigger.whileActiveContinuous(Commands.trackTarget(m_turret, Pipeline.POWER_CELLS, () -> m_drivetrain.getPose()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }

  public void resetSubsystems() {
    m_drivetrain.resetPose();
  }

  private double getMove() {
    double move = -m_extreme.getStickY(2);
    move = Math.abs(Math.pow(move, 2)) * Math.signum(move);
    return move / 2;
  }

  private double getTurn() {
    double turn = -m_extreme.getStickRotation(2);
    turn = Math.abs(Math.pow(turn, 2)) * Math.signum(turn);
    return turn / 2;
  }

  public void setDriveType(DriveType driveType) {
    Command driveCommand = null;
    switch (driveType) {
      case SIMPLE_ARCADE_DRIVE:
        driveCommand = Commands.simpleArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn());
        break;
      case VELOCITY_ARCADE_DRIVE:
        driveCommand = Commands.velocityArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn());
        break;
      case ACCELERATION_LIMITED_SIMPLE_ARCADE_DRIVE:
        driveCommand = Commands.accelerationLimitedSimpleArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn(), 2, 3);
        break;
      case ACCELERATION_LIMITED_VELOCITY_ARCADE_DRIVE:
        driveCommand = Commands.accelerationLimitedVelocityArcadeDrive(m_drivetrain, () -> getMove(), () -> getTurn(), 2, 3);
      default:
        break;
    }
    m_drivetrain.setDefaultCommand(driveCommand);
  }
}
