// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.InputMode;

public class HybridPosition extends CommandBase {
  private Intake intake;
  private InputMode mode;

  /** Creates a new HybridPosition. */
  public HybridPosition(Intake intake, InputMode mode) {
    this.intake = intake;
    this.mode = mode;
    addRequirements(intake, mode);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.mode.Mode == true) {
      this.intake.OperatorCubeSpeed(-0.9);
      this.intake.OperatorCubeDegrees(75);
      SmartDashboard.putString("Output", "Cube Hybrid Position");
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
