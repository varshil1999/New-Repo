// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntakeAngle extends CommandBase {
  private Intake intake;
  double angle;
  boolean flag;
  /** Creates a new SetIntakeAngle. */
  public SetIntakeAngle(Intake intake) {

    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.intake.OperatorCubeDegrees(50);
    this.intake.DriverCubeDegrees();
    if (this.intake.PositionIntake() >= 49) {
      flag = true;
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flag;
  }
}
