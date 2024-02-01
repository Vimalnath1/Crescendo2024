// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends Command {
  /** Creates a new DefaultDrive. */
  private final DriveSubsystem drivetrain;
  private final DoubleSupplier xvalue;
  private final DoubleSupplier yleftvalue;
  private final DoubleSupplier yrightvalue;
  private final DoubleSupplier turnvalue;
  public DefaultDrive(DriveSubsystem subsystem,DoubleSupplier x,DoubleSupplier ybackward,DoubleSupplier yforward, DoubleSupplier turn) {
    drivetrain=subsystem;
    xvalue=x;
    yleftvalue=ybackward;
    yrightvalue=yforward;
    turnvalue=turn;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (yrightvalue.getAsDouble()>0){
    //   drivetrain.drive(-yrightvalue.getAsDouble(), xvalue.getAsDouble(), turnvalue.getAsDouble(), false, true);
    // }
    // if (yleftvalue.getAsDouble()>0){
    //   drivetrain.drive(yleftvalue.getAsDouble(), xvalue.getAsDouble(), turnvalue.getAsDouble(), false, true);
    // }
    double yvalue=yrightvalue.getAsDouble()+yleftvalue.getAsDouble();
    // if (yrightvalue.getAsDouble()==0 && yleftvalue.getAsDouble()==0){
      drivetrain.drive(-yvalue, xvalue.getAsDouble(), turnvalue.getAsDouble(), false, true);
      SmartDashboard.putNumber("Trigger Value", yvalue);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
