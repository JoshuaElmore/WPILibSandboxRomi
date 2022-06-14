// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.utils.XboxControllerHelper;
import frc.robot.commands.ResetGyroCommand;
import frc.robot.commands.DriveStraightCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.commands.TurnCommand;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController driveController = new XboxController(Constants.DRIVE_CONTROLLER_PORT);
  private final XboxControllerHelper driveControllerHelper = new XboxControllerHelper(driveController);

  private RomiDrivetrain m_romiDrivetrain;

  private TankDriveCommand defaultDriveCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    createSubsystems(); // Create our subsystems.
    createCommands(); // Create our commands
    configureButtonBindings(); // Setup our button bindings
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void createSubsystems(){
    m_romiDrivetrain = new RomiDrivetrain();
  }

  private void createCommands(){
    defaultDriveCommand = new TankDriveCommand(
      m_romiDrivetrain,
      () -> driveControllerHelper.scaleAxis(driveController.getRightY()),
      () -> driveControllerHelper.scaleAxis(driveController.getLeftY())
      );
    m_romiDrivetrain.setDefaultCommand(defaultDriveCommand);
  }

  private void configureButtonBindings() {
    new JoystickButton(driveController, Button.kA.value).whenPressed(new DriveStraightCommand(m_romiDrivetrain, 10.0, 0.6));

    new JoystickButton(driveController, Button.kX.value).whenPressed(new TurnCommand(m_romiDrivetrain, -90.0));

    new JoystickButton(driveController, Button.kStart.value).whenPressed(new ResetGyroCommand(m_romiDrivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
