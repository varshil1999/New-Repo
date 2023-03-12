// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCube extends CommandBase {
  Intake intake;
  double angle;
  boolean flag;
  double lastTimeStamp,timer=0,increment=0;
  /** Creates a new IntakeCube. */
  public IntakeCube(Intake intake, double angle) {
    this.intake = intake;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
    timer = Timer.getFPGATimestamp();
    increment=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (cubeSubsystem.getLimitSwitch() == true){
    //   cubeSubsystem.resetCubeIntakeAngle();
    // }
    // else {
      
      
      lastTimeStamp = (Timer.getFPGATimestamp()-timer);
      this.intake.OperatorCubeDegrees(115);
      this.intake.DriverCubeDegrees();
      this.intake.OperatorCubeSpeed(0.3);
      this.intake.DriverCubeSpeed();
     if (this.intake.detectBeamBreaker1() == true || lastTimeStamp>2) {
        this.intake.OperatorCubeSpeed(0);
        this.intake.DriverCubeSpeed();
        this.intake.OperatorCubeDegrees(angle);
        this.intake.DriverCubeDegrees();
        flag = true;
      }}
      
      
      // if (cubeSubsystem.getCubeUpDownAngle()>= 75) flag = true;

      

      // else if (cubeSubsystem.getBeamBreakerCube() == false) {
        // cubeSubsystem.setCubeIntakeAngle(117);
        // cubeSubsystem.intakeCube(-0.2);
      // }
    // }    

    // cubeSubsystem.intakeCube(0.2);
    // cubeSubsystem.setCubeIntakeAngle(50);
    
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (flag ==true){
    //  cubeSubsystem.intakeCube(0);
    //   return true;
    // }
    // else return false;
      if(flag==true){
      return true;

  }
  else{
    return false;
  }
  }
}