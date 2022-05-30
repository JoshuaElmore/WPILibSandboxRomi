// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveStraightCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final RomiDrivetrain romiDrivetrain;
  private PIDController PIDController = new PIDController(1.0, 0.0, 0.0);
  private final Double length;
  private double threshold = 0.5;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraightCommand(RomiDrivetrain subsystem, Double length, Double power) 
  {
    romiDrivetrain = subsystem;
    this.length = length;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    romiDrivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    //If we reached the destinasion
    romiDrivetrain.setBothPower(MathUtil.clamp(PIDController.calculate(romiDrivetrain.getAvgDistanceInch(), length),-1,1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(romiDrivetrain.getAvgDistanceInch()-length)<threshold);
  }
}
