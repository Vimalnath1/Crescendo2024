// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSourceSide extends SequentialCommandGroup {
  /** Creates a new AutoSourceSide. */
  public AutoSourceSide(DriveSubsystem drivetrain,Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootRing(shooter, 0.5).withTimeout(2),
      new DefaultDrive(drivetrain,()->0,()->0.3,()->0,()->0).withTimeout(3.5)
    );
  }
}
