// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutofromCenter extends SequentialCommandGroup {
  /** Creates a new AutofromCenter. */
  public AutofromCenter(DriveSubsystem drivetrain,Shooter shooter,Feeder feeder,ExampleSubsystem sub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
      
      // new ExampleCommand(sub).withTimeout(0.75),
      new DefaultDrive(drivetrain,()->0,()->-0.3,()->0).withTimeout(0.75),
      new DefaultDrive(drivetrain,()->0,()->-0,()->0).withTimeout(0.5),
      // new ExampleCommand(sub).withTimeout(5)
      
      // new ShootRing(shooter, 1),
      new SpeedUptoShoot(shooter, feeder, 1),
      new DefaultDrive(drivetrain,()->0,()->-0.3,()->0).withTimeout(0.5)
    );
  }
}
