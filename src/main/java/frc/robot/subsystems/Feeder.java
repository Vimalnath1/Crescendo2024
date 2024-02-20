// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  CANSparkMax feeder;
  public Feeder() {
    feeder=new CANSparkMax(13, MotorType.kBrushed);
  }
  public void feed(double speed){
    feeder.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
