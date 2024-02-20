// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax leftshooter;
  CANSparkMax rightshooter;
  public Shooter() {
    leftshooter=new CANSparkMax(9, MotorType.kBrushless);
    rightshooter=new CANSparkMax(10, MotorType.kBrushless);
  }
  public void shoot(double speed){
    leftshooter.set(speed);
    rightshooter.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
