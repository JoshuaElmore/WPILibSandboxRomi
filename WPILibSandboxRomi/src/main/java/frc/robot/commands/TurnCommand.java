// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class TurnCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final RomiDrivetrain romiDrivetrain;
  private final Double angel;

  private PIDController anglePIDController = new PIDController(0.005, 0.0, 0.0);
  private double anglePIDValue = 0.0;
  private double angleThreshold = 1.5;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnCommand(RomiDrivetrain subsystem, Double angel) 
  {
    romiDrivetrain = subsystem;
    this.angel = angel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    romiDrivetrain.resetGyro();
    anglePIDController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
  anglePIDValue = MathUtil.clamp(anglePIDController.calculate(romiDrivetrain.getAzimuth(), angel),-1,1);
      romiDrivetrain.setLeftPower(anglePIDValue);
      romiDrivetrain.setRightPower(-anglePIDValue);
      System.out.println(angel +" , "+romiDrivetrain.getAzimuth()+" , "+ anglePIDValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(Math.abs(romiDrivetrain.getAzimuth()-angel)<angleThreshold);
  }
}
