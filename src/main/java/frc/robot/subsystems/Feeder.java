// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  CANSparkMax feeder;

  // AbsoluteEncoder feederencoder;
  // SparkPIDController feederPIDController;
  // PIDController feedercontrol;
  // DigitalInput leftlimitswitch;
  // DigitalInput rightlimitswitch;

  Spark feeder1;
  public Feeder() {
    // feederspark
    feeder=new CANSparkMax(13, MotorType.kBrushed);
    // leftlimitswitch=new DigitalInput(0);
    // rightlimitswitch=new DigitalInput(2);
    // rightlimitswitch=new DigitalInput(1);
    // feederencoder=feeder.getAbsoluteEncoder(Type.kDutyCycle);
    // feedercontrol=new PIDController(0.5, ModuleConstants.kTurningI, ModuleConstants.kTurningD);
    // feederPIDController.setFeedbackDevice(feederencoder);

    // feederencoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    // feederencoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    
    // feederPIDController.setP(ModuleConstants.kTurningP);
    // feederPIDController.setI(ModuleConstants.kTurningI);
    // feederPIDController.setD(ModuleConstants.kTurningD);
    // feederPIDController.setFF(ModuleConstants.kTurningFF);
    // feederPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    // feeder.burnFlash();
    feeder1=new Spark(0);
  }
  public void feed(double speed){
    // if (leftlimitswitch.get()==true && rightlimitswitch.get()==true){
      feeder.set(speed);
      feeder1.set(speed);
    // }
    // else if (leftlimitswitch.get()==false){
    //   if (speed>0){
    //     feeder.set(0);
    //     feeder1.set(0);
    //   }
    //   else{
    //     feeder.set(speed);
    //     feeder1.set(speed);
    //   }
    // }
    // else if (rightlimitswitch.get()==false){
    //   if (speed<0){
    //     feeder.set(0);
    //     feeder1.set(0);
    //   }
    //   else{
    //     feeder.set(speed);
    //     feeder1.set(speed);
    //   }
    // }
    // feederPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
    
    // feeder1.set(speed);
  }
  public void feedwithencoder(double angle){
    //0.165
    // double error=feederencoder.getPosition()-angle;
    // // double speed=feedercontrol.calculate(error);
    // if (error>0.1){
    //   feeder.set(0.5);
    // }
    // else if (error<-0.1){
    //   feeder.set(-0.5);
    // }
    // feederPIDController.setReference(error, CANSparkMax.ControlType.kPosition);
    // System.out.println(speed);
    // feeder.set(speed);
    // feederPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
