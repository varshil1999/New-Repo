// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.InputMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;

public class IntakePosition extends CommandBase {
  private Intake intake;
  private InputMode mode;
  private boolean intakeflag, Groundflag;
  private Arm arm;
  private Gripper grip;
  double GroundCount = 0;
  boolean flag = false, once = true, Substationonce = true,Groundonce=true, beambreaker = true, returnflag = false, Groundreturnflag = false;
  int count = 0;
  double armpos1,elbowpos1,armpos2,elbowpos2;

  /** Creates a new IntakePosition. */
  public IntakePosition(Intake intake, InputMode mode, Arm arm, Gripper grip) {
    this.intake = intake;
    this.mode = mode;
    this.arm = arm;
    this.grip = grip;
    addRequirements(intake, mode, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Groundonce=true;
    intakeflag = true;
    count = 0;
    GroundCount=0;
    // Groundflag = true;
    returnflag = false;
    Groundreturnflag =false;
    once = true;
    flag = true;
    armpos1=218.36;//165
    elbowpos1=294.67;//-37
    armpos2=238.36;//185;
    elbowpos2=292.17;//-35.5;
    this.grip.IsGrip(true);
    this.grip.Grip();
    this.intake.OperatorCubeSpeed(0.3);
    this.intake.OperatorCubeDegrees(117);
    SmartDashboard.putString("Check Loop","0");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putBoolean("flag", flag);
    SmartDashboard.putNumber("count", count);
    SmartDashboard.putBoolean("RETUNflag", returnflag);
    SmartDashboard.putBoolean("Groundflag", Groundflag);

    if (this.mode.Mode == true) {
      if (intakeflag == true) {
        if (this.intake.detectBeamBreaker1() == false) {
          this.intake.DriverCubeDegrees();
          if (this.intake.PositionIntake() >= 80)
            this.intake.DriverCubeSpeed();
          SmartDashboard.putString("Output", "Cube Ground Position");
          SmartDashboard.putString("Check Loop","1");
          return;
        } else if (this.intake.detectBeamBreaker1() == true) {
          this.intake.IntakeCube(0);
          this.intake.OperatorCubeDegrees(30);
          this.intake.DriverCubeDegrees();
          intakeflag = false;
          SmartDashboard.putString("Output", "Cube BEamBreakr breaked ");
          SmartDashboard.putString("Check Loop","2");
          return;
        }
      }
    }else if (this.mode.Mode == false){
      if(this.arm.GroundType()==false){
        SmartDashboard.putNumber("Groundcount", GroundCount);
      
      if ( GroundCount ==0 && Groundonce){
        this.intake.OperatorCubeDegrees(15);
        this.intake.DriverCubeDegrees();
        // this.intake.OperatorCubeSpeed(-0.25);
        this.arm.SetOperatorArmCancoderValues(261.485);//208.125
        this.arm.SetOperatorELbowCancoderValues(276.38);//-55.84
        this.arm.setArmcancoderDegrees();
        this.arm.setElbowcancoderDegrees();
        GroundCount=1;
        Groundonce = false;
        flag=false;
        
        SmartDashboard.putString("Check Loop","3");
      }
      else if(flag == true && GroundCount == 1){
        this.arm.SetOperatorArmCancoderValues(261.485);//208.125
        this.arm.SetOperatorELbowCancoderValues(310.38);//-17.84
        // this.intake.OperatorCubeDegrees(15);
        // this.intake.DriverCubeDegrees();
        this.arm.setArmcancoderDegrees();
        this.arm.setElbowcancoderDegrees();
        GroundCount=2;
        flag = false;
        }
      

       else if (flag == true && GroundCount==2) {
        this.arm.GripperDegrees(135);
        this.arm.GripperRotate();
        
          GroundCount = 3;
          SmartDashboard.putString("Check Loop","4");
        }
  
         else if (flag == true && GroundCount == 3) {
          this.arm.SetOperatorArmCancoderValues(armpos2);
          this.arm.SetOperatorELbowCancoderValues(elbowpos2);
          this.arm.setArmcancoderDegrees();
          this.arm.setElbowcancoderDegrees();
          
       
          flag =false;
          
          GroundCount=4;

          SmartDashboard.putString("Check Loop","5");
        }

        else if (flag == true && GroundCount == 4) {
          
          if(this.arm.GripperBeamBreaked()==true){
            this.grip.IsGrip(false);
            this.grip.Grip();
            this.arm.GripperDegrees(-10);
            this.arm.GripperRotate();

           GroundCount=5;
           
          }
        }
        
          else if(GroundCount==5){
            this.arm.SetOperatorArmCancoderValues(261.485);//208.125
            this.arm.SetOperatorELbowCancoderValues(314.38);//-17.84
            this.arm.setArmcancoderDegrees();
            this.arm.setElbowcancoderDegrees();
            flag=false;
            GroundCount=6;
          }
          else if(GroundCount==6&& flag == true){
            this.arm.SetOperatorArmCancoderValues(261.485);//208.125
            this.arm.SetOperatorELbowCancoderValues(276.38);//-55.84
            this.arm.setArmcancoderDegrees();
            this.arm.setElbowcancoderDegrees();
            flag=false;
            GroundCount=7;
          }
          else if(GroundCount==7&& flag==true){
            this.arm.SetOperatorArmCancoderValues(222.36);//169
            this.arm.SetOperatorELbowCancoderValues(219.55);//-112.67
            this.arm.setArmcancoderDegrees();
            this.arm.setElbowcancoderDegrees();
            flag=false;
            GroundCount=8;
          }
          else if(GroundCount==8 && flag==true){
            this.intake.OperatorCubeDegrees(15);
            this.intake.DriverCubeDegrees();
         
              returnflag=true;
            }
          
          }
    }
    if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
    && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200)) {
    flag = true;
    SmartDashboard.putString("Check Loop","13");
    
    } else {
  flag = false;
  SmartDashboard.putString("Check Loop","14");
      
  }   
    SmartDashboard.putBoolean("flag", flag);
    SmartDashboard.putBoolean("RETUNflag", returnflag);                             
    SmartDashboard.putBoolean("Groundflag", Groundflag);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return returnflag;
  }
}
