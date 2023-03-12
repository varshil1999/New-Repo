// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DockBalance3 extends CommandBase {
  /** Creates a new DockBalance3. */
  public Swerve drivetrain;
  private int pauseCounter;
  public int state=0;
  boolean returnflag=false;
  private static final double APPROACH_SPEED = -0.3;
  private Rotation2d tiltAngle;
  public DockBalance3(Swerve drivetrain) {
    this.drivetrain=drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pauseCounter = 0;
    returnflag=false;
    state=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Doclbalancestate", state);
    tiltAngle = drivetrain.getTilt();
    if (state==0){
      drivetrain.drive( new Translation2d(APPROACH_SPEED*Constants.Swerve.maxSpeed,0), 
    0 , 
    !false, 
    true);
    state=1;
    }
    else if(state==1){
      if( Math.abs(tiltAngle.getDegrees()) >12){
          state=2;
      }

    }
    else if(state==2){
      drivetrain.drive( new Translation2d((-0.4)*Constants.Swerve.maxSpeed,0), 
      0 , 
      !false, 
      true);
      Timer.delay(0.5);
      state=3;

    }
    else if (state==3){
      if(Math.abs(tiltAngle.getDegrees()) <10){

        drivetrain.drive( new Translation2d((0.3)*Constants.Swerve.maxSpeed,0), 
        0 , 
        !false, 
        true);
Timer.delay(0.3);
        state=4;
        returnflag=true;
      }
    }
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return returnflag;
  }
}
