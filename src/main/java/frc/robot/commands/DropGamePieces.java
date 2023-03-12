// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.InputMode;;

public class DropGamePieces extends CommandBase {
  private Intake intake;
  private InputMode mode;
  private double timer, lasttimestamp = 0;
  private Gripper grip;
  private Arm arm;
  private boolean returnflag;
  /** Creates a new MediumPosition. */
  public DropGamePieces(Intake intake, InputMode mode,Arm arm,Gripper grip) {
    this.intake = intake;
    this.mode = mode;
    this.grip=grip;
    this.arm=arm;
    addRequirements(intake,mode);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer = Timer.getFPGATimestamp();
    returnflag=false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.mode.Mode==true){
      this.intake.DriverCubeSpeed();
      SmartDashboard.putString("OutputPower", "Cube Speed");
      // this.lasttimestamp = (Timer.getFPGATimestamp() - this.timer);
    }
    else if(this.mode.Mode == false){

      this.grip.Grip();
      this.arm.GripperDegrees(-6);
      this.arm.GripperRotate();
      this.arm.HomePostion();
    }
    this.lasttimestamp = (Timer.getFPGATimestamp() - this.timer);
      
    if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
    && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200)) {
      // this.intake.OuttakeCube(0);
      this.grip.IsGrip(false);
      this.grip.Grip();

      returnflag=true;
} 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.mode.Mode==true){
    if (this.lasttimestamp > 1){
  
    this.intake.OuttakeCube(0);
    this.intake.OperatorCubeDegrees(15);
    this.intake.DriverCubeDegrees();
    return true;
  }
  else{
    return false;
  }
  }
  else if(this.mode.Mode == false){ 
    this.intake.OperatorCubeDegrees(15);
    this.intake.DriverCubeDegrees();
    return returnflag;
  }
  else{
    return false;
  }
}

}
