// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax leftclimber;
  CANSparkMax rightclimber;
  public Climber() {
    leftclimber=new CANSparkMax(11, MotorType.kBrushless);
    rightclimber=new CANSparkMax(12, MotorType.kBrushless);
  }

  public void climb(double speed){
    leftclimber.set(speed);
    rightclimber.set(-speed);
  }
  
  public void oneclimber(double leftspeed,double rightspeed){
    leftclimber.set(leftspeed);
    rightclimber.set(-rightspeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }




}
