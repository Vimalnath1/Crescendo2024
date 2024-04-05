// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

public class OneClimb extends Command {
  /** Creates a new OneClimb. */
  Climber climber;
  private double rightspeed=0;
  private double leftspeed=0;
  public OneClimb(Climber subsystem,double rs,double ls) {
    climber=subsystem;
    rightspeed=rs;
    leftspeed=ls;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.oneclimber(leftspeed, rightspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.oneclimber(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
