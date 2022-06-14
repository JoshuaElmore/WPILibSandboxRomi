// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/** An example command that uses an example subsystem. */
public class TurnCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  private final RomiDrivetrain romiDrivetrain;
  private Double angel;
  private final Double howMuchToTurn;

  private PIDController anglePIDController = new PIDController(0, 0, 0);
  private double anglePIDValue = 0.0;
  private double angleThreshold = 1.5;
  private double angularVelocityThreshold = 5;
  private int exitCount = 0;

  NetworkTableEntry goalHeading;
  NetworkTableEntry kp;
  NetworkTableEntry ki;
  NetworkTableEntry kd;
  NetworkTableEntry done;
  NetworkTableEntry error;
  NetworkTableEntry PIDOut;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnCommand(RomiDrivetrain subsystem, Double angel) {
    romiDrivetrain = subsystem;
    this.howMuchToTurn = angel*0.96;
    initTelemetry();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  private void initTelemetry() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drive");

    goalHeading = tab.add("Goal Angel", 0)
        .withPosition(1, 0)
        .withSize(1, 1)
        .getEntry();

    kp = tab.add("kP", 0.013)
        .withPosition(0, 1)
        .withSize(1, 1)
        .getEntry();

    ki = tab.add("kI", 0.0002)
        .withPosition(0, 2)
        .withSize(1, 1)
        .getEntry();

    kd = tab.add("kD", 0.0012)
        .withPosition(0, 3)
        .withSize(1, 1)
        .getEntry();

    done = tab.add("Done", false)
        .withPosition(2, 0)
        .withSize(1, 1)
        .getEntry();

    error = tab.add("Angel Error", 0)
        .withPosition(3, 0)
        .withSize(1, 1)
        .getEntry();
      
    PIDOut = tab.add("PID output", 0)
        .withPosition(4, 0)
        .withSize(1, 1)
        .getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angel = romiDrivetrain.getAzimuth() + howMuchToTurn;
    anglePIDController.reset();
    anglePIDController.setTolerance(angleThreshold, angularVelocityThreshold);
    done.forceSetBoolean(false);

    anglePIDController.setP(kp.getDouble(-1));
    anglePIDController.setI(ki.getDouble(-1));
    anglePIDController.setD(kd.getDouble(-1));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    anglePIDValue = anglePIDController.calculate(romiDrivetrain.getAzimuth(), angel);
    anglePIDValue = MathUtil.clamp(anglePIDValue, -1, 1);
    PIDOut.forceSetNumber(anglePIDValue);
    romiDrivetrain.setRightPower(-anglePIDValue);
    romiDrivetrain.setLeftPower(anglePIDValue);

    if (anglePIDController.atSetpoint()) {
      exitCount += 1;

      if (exitCount == 2) {
        done.forceSetBoolean(true);
      }

    } else {
      exitCount = 0;
    }
    error.setNumber(anglePIDController.getPositionError());
    goalHeading.setNumber(angel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (exitCount >= 2) {
      return true;
    }
    return false;
  }
}
