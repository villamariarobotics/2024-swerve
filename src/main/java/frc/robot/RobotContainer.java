// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SendableChooser<Command> m_autoChooser;

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    defaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Controller - Right Trigger makes wheels go to X-formation
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

  }

  private void defaultCommands() {

    m_robotDrive.setDefaultCommand(
        new RunCommand(() -> {
          if (m_driverController.getAButton()) {
            m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    Math.signum(m_driverController.getLeftY()) * Math.pow(m_driverController.getLeftY(), 2),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    Math.signum(m_driverController.getLeftX()) * Math.pow(m_driverController.getLeftX(), 2),
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX() + m_driverController.getRightTriggerAxis()
                    - m_driverController.getLeftTriggerAxis()) * 1, OIConstants.kDriveDeadband),
                false, true);
          } else {
            m_robotDrive.drive(
                -MathUtil.applyDeadband(
                    Math.signum(m_driverController.getLeftY()) * Math.pow(m_driverController.getLeftY(), 2) * 0.5,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(
                    Math.signum(m_driverController.getLeftX()) * Math.pow(m_driverController.getLeftX(), 2) * 0.5,
                    OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController.getRightX() + m_driverController.getRightTriggerAxis()
                    - m_driverController.getLeftTriggerAxis()) * 1, OIConstants.kDriveDeadband),
                false, true);
          }
        }, m_robotDrive));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * '
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = m_autoChooser.getSelected();
    autoCommand.addRequirements(m_robotDrive);
    return autoCommand;

    // return new PivotGoToSpeakerPositionCommand(m_pivotSubsystem).andThen(
    // new AutoShootingNote(m_endEffectorSubsystem).andThen(
    // new PivotGoToAmpPositionCommand(m_pivotSubsystem)
    // )
    // );
  }
}