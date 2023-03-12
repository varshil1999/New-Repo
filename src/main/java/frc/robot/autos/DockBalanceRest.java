// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Command which balances the robot on the charge station at the end of auto.
 */
public class DockBalanceRest extends CommandBase {

  private static final double BALANCE_THRESHOLD = 0.1; // degrees, "balanced" if within +/- BALANCE_THRESHOLD.
  
  private Swerve drivetrain;

  // private int pauseCounter;
  private Rotation2d tiltAngle;
  double maximumspeed=0.7;
  PIDController balancePID;
  double capspeed=0.1;

  /** Creates a new AutoBalanceOnChargeStation command. */
  public DockBalanceRest(Swerve drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancePID=new PIDController(0.025, 0, 0.004);
    balancePID.setTolerance(2);
    // balancePID.enableContinuousInput(-Math.PI, Math.PI);
    balancePID.setSetpoint(0);
   
    // previousSpeed = 0;
    // pauseCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tiltAngle = drivetrain.getTilt();
    double currentSpeed;

    if(Math.abs(tiltAngle.getDegrees()) <= BALANCE_THRESHOLD){
      currentSpeed=0;
    }
    else{

      currentSpeed = -balancePID.calculate(tiltAngle.getDegrees());
      if(currentSpeed>capspeed){
        currentSpeed=capspeed;
      }
      else if(currentSpeed<-capspeed){
        currentSpeed=-capspeed;
      }
    

    }

    
    // currentSpeed*=maximumspeed;
    SmartDashboard.putNumber("Dockbalance speed", currentSpeed*Constants.Swerve.maxSpeed);
    drivetrain.drive( new Translation2d(currentSpeed*Constants.Swerve.maxSpeed,0), 
    0 , 
    !false, 
    true);
    // previousSpeed = currentSpeed;
  }

  // proportionally map the tilt angle to speed
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return  Math.abs(tiltAngle.getDegrees()) <= BALANCE_THRESHOLD; // ends command if balanced for
                                                                                       // 1s
  }
}
