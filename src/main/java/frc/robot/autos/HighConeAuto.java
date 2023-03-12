// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.InputMode;
import frc.robot.subsystems.Intake;

public class HighConeAuto extends CommandBase {
  private Intake intake;
  private InputMode mode;
  private Arm arm;
  private boolean  Mediumflag;
  private double Mediumcount;
  private boolean returnflag;
  private Gripper grip;
  double armpos1,elbowpos1,armpos2,elbowpos2;
  /** Creates a new HighConeAuto. */
  
  public HighConeAuto (Intake intake, Arm arm, Gripper grip) {
    this.intake = intake;
    this.mode = mode;
    this.arm=arm;
    this.grip=grip;
    addRequirements(intake,grip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    this.grip.IsGrip(false);
    this.grip.Grip();
    returnflag =false;
    Mediumflag =true;
    Mediumcount = 0;
    armpos1=221.36;//168
    elbowpos1=300.22;//-32
    armpos2=181.83;//128.47
    elbowpos2=315.21;//-7
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ( Mediumcount == 0) {
      this.intake.OperatorCubeDegrees(50);
      this.intake.DriverCubeDegrees();
      Mediumcount= 0.25;
     
    }

    if( this.intake.PositionIntake() >= 45 ){ 
      if (Mediumflag == true && Mediumcount< 1) {

        this.arm.SetOperatorArmCancoderValues(armpos1);
        this.arm.SetOperatorELbowCancoderValues(elbowpos1);
        this.arm.setArmcancoderDegrees();
        this.arm.setElbowcancoderDegrees();
        Mediumcount = 1;
        Mediumflag =false;
        
      } else if (Mediumflag == true && Mediumcount == 1) {
        this.arm.SetOperatorArmCancoderValues(armpos2);
        this.arm.SetOperatorELbowCancoderValues(elbowpos2);
        this.arm.GripperDegrees(140);
        this.arm.GripperRotate();
        this.arm.setArmcancoderDegrees();
        this.arm.setElbowcancoderDegrees();
        Mediumcount = 2;
      } else if(Mediumflag == true && Mediumcount == 2){
        returnflag=true;
      }
  }
  if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
          && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200)) {
        Mediumflag = true;
      }
      
      else {
        Mediumflag = false;
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
