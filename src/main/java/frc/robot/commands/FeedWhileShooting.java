// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedWhileShooting extends ParallelCommandGroup {
  /** Creates a new FeedWhileShooting. */
  public FeedWhileShooting(Shooter shooter,Feeder feeder,double speed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (speed>0){
    addCommands(
      new ShootRing(shooter, speed).withTimeout(1),
      new FeedRing(feeder,-1).withTimeout(0.25) //May have to negate and change times
    );
    }
    else{
      addCommands(
      new ShootRing(shooter, speed).withTimeout(1),
      new FeedRing(feeder,1).withTimeout(0.25) //May have to negate and change times
    );
    }
  }
}
