// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import frc.robot.subsystems.Arm;
import frc.robot.subsystems.InputMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Intake;

public class SubstationPosition extends CommandBase {
  private Arm arm;
  private InputMode mode;
  /** Creates a new IntakePosition. */
  public SubstationPosition(Arm arm, InputMode mode) {
    this.arm = arm;
    this.mode = mode;
    addRequirements(arm,mode);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  if(this.mode.Mode == false){
    this.arm.InputSUBORGRO(false);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
