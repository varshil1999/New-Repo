// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import frc.robot.subsystems.Arm;
import frc.robot.subsystems.InputMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class GroundPosition extends CommandBase {
  private Intake intake;
  private InputMode mode;
  private Arm arm;

  /** Creates a new IntakePosition. */
  public GroundPosition(Intake intake, InputMode mode, Arm arm) {
    this.intake = intake;
    this.mode = mode;
    this.arm = arm;
    addRequirements(intake, mode, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.mode.Mode==true){
      this.intake.OperatorCubeSpeed(0.3);
      this.intake.OperatorCubeDegrees(115); 
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
