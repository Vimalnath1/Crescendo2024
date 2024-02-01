// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.*;

public class LineUptoTag extends Command {
  /** Creates a new LineUptoTag. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  DriveSubsystem driveTrain;
  NetworkTableEntry tx=table.getEntry("tx");
  NetworkTableEntry ty=table.getEntry("ty");
  double xoffset;
  double yOffset;
  double xKp = 0.1; //Subject to change
  double xKi=0;
  double xKd=0;
  double xSpeed;
  double zdistancefromtag;
  double xdistancefromtag;
  private PIDController xcontroller;

  public LineUptoTag(DriveSubsystem subsystem) {
    driveTrain=subsystem;
    addRequirements(driveTrain);
    xcontroller=new PIDController(xKp,xKi,xKd);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Update_Limelight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Update_Limelight();
    // CenterRobot(driveTrain);
    // driveTrain.drivedistance(Units.inchesToMeters(zdistancefromtag*0.5), 1);
  }

  public void Update_Limelight(){
    double tv=table.getEntry("tv").getDouble(0.0);
    if (tv!=0){
    double idnum= table.getEntry("tid").getDouble(0.0);
    SmartDashboard.putNumber("April Tag Id", idnum);
      xoffset=tx.getDouble(0);
      yOffset=ty.getDouble(0);
      zdistancefromtag=getDistancefromTag(8.5, 50, 15.5, yOffset);//in inches, subject to change
      xdistancefromtag=zdistancefromtag*Math.tan(Math.toRadians(xoffset));
      SmartDashboard.putNumber("LimelightX", xoffset);
      SmartDashboard.putNumber("Z-distance", zdistancefromtag);
      SmartDashboard.putNumber("X-distance", xdistancefromtag);
      
      

      // if (xoffset>1 || xoffset<-1){
      // xSpeed=xcontroller.calculate(xoffset);
      // SmartDashboard.putNumber("Speed", xSpeed);
      // }
      // else{
      //   xSpeed=0;
      // }
      //driveTrain.drive(0, -xSpeed, 0, false,true);
    }
  }
  public double getDistancefromTag(double heightOfLimelight, double heightOfGoal, double angleOfLimelight,double yOffset){
    yOffset+=angleOfLimelight;
    yOffset=Math.toRadians(yOffset);
    
    return (heightOfGoal-heightOfLimelight)/Math.tan(yOffset);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0, 0, false,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
