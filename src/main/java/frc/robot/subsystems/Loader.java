// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {
  /** Creates a new Loader. */
  // CANSparkMax loader1;
  // CANSparkMax loader2;
  public Loader() {
    // loader1=new CANSparkMax(9, MotorType.kBrushed);
    // loader2=new CANSparkMax(10, MotorType.kBrushed);
  }

  public void load(double speed){
    // loader1.set(speed);
    // loader2.set(speed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
