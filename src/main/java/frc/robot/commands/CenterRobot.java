// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class CenterRobot extends Command {
  /** Creates a new CenterRobot. */
  DriveSubsystem driveTrain;
  // ADIS16470_IMU gyroscope;
  private PIDController turnpid;
  double kPThetaController=1;
  double kIThetaController=0;
  double kDThetaController=0;
  public CenterRobot(DriveSubsystem subsystem) {
    driveTrain=subsystem;
    addRequirements(driveTrain);
    
    turnpid = new PIDController(kPThetaController, kIThetaController, kDThetaController);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle=Math.toRadians(driveTrain.m_gyro.getAngle(ADIS16470_IMU.IMUAxis.kYaw)*(Math.PI/180));
    double speed=turnpid.calculate(angle);
    // double speed=1.0;
    driveTrain.drive(0, 0, speed, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
