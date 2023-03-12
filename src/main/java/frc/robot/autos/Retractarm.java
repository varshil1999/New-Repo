// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.InputMode;
import frc.robot.subsystems.Intake;

public class Retractarm extends CommandBase {
  private Intake intake;
  

  private Gripper grip;
  private Arm arm;
  private boolean returnflag;

  /** Creates a new Retractarm. */
  public Retractarm(Intake intake, Arm arm,Gripper grip) {
    this.intake = intake;
    this.grip=grip;
    this.arm=arm;
    addRequirements(intake,arm,grip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    returnflag=false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.grip.IsGrip(true);
    this.grip.Grip();
    this.arm.GripperDegrees(-6);
    this.arm.GripperRotate();
    this.arm.HomePostion();
    if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
    && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200)) {
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
    this.intake.OperatorCubeDegrees(15);
    this.intake.DriverCubeDegrees();
    return returnflag;
  }

}