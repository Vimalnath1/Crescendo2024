// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTest extends SequentialCommandGroup {
  /** Creates a new AutoTest. */
  public AutoTest(DriveSubsystem drivetrain,Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LineUptoTag(drivetrain),//Amp
      new DriveDistance(drivetrain, 2, 1),
      new DefaultDrive(drivetrain, ()->0, ()->0, ()->0, ()->1).withTimeout(0.25),
      new LineUptoTag(drivetrain),
      new ShootRing(shooter, 0.5)

    );
  }
}
