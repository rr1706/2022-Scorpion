// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimAtTarget;
import frc.robot.commands.DriveByController;
import frc.robot.subsystems.Swerve.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private final Drivetrain m_drive = new Drivetrain();

  private final DriveByController m_driveByController = new DriveByController(m_drive, m_driverController);
  private final AimAtTarget m_aimAtHub = new AimAtTarget(m_drive, m_driverController, GoalConstants.kGoalLocation, true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(m_driveByController);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new POVButton(m_driverController, 0)
        .whenPressed(() -> m_drive.resetOdometry(new Rotation2d(0.0)));

    

    new JoystickButton(m_driverController, Button.kY.value).whenPressed(()->m_drive.resetOdometry(new Pose2d(3.89,5.41, m_drive.getGyro().times(-1.0))));
    new JoystickButton(m_driverController, Button.kA.value).whileHeld(m_aimAtHub);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new WaitCommand(20.0);
  }
}
