// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mover extends SubsystemBase {
  /** Creates a new Mover. */
  CANSparkMax mover;
  public Mover() {
    // mover=new CANSparkMax(14, MotorType.kBrushless);

  }

  public void move(double speed){
    // mover.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
