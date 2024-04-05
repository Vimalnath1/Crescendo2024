// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  
  private final DoubleSolenoid leftpiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 0);
  private final DoubleSolenoid rightpiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 7);
  
  public Pneumatics() {
    // piston1.set(DoubleSolenoid.Value.kOff);
    // piston2.set(DoubleSolenoid.Value.kOff);
    leftpiston.set(DoubleSolenoid.Value.kForward);
    rightpiston.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void runpistonforward(){
    // piston1.set(DoubleSolenoid.Value.kForward);
    leftpiston.toggle();
    rightpiston.toggle();
    // piston2.set(DoubleSolenoid.Value.kForward);
  }
  public void runpistonbackward(){
    // piston1.set(DoubleSolenoid.Value.kReverse);
    // piston2.set(DoubleSolenoid.Value.kReverse);
  }
}
