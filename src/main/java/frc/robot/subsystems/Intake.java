// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.InaccessibleObjectException;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  TalonFX LeftIntake;
  TalonFX RightIntake;
  TalonFX Intake;
  double pos, increment = 0;
  // DigitalInput BINTAKE; 
  boolean Bintake,Barm;
  double Cubedegrees;
  double CubeSpeed;
  public static DigitalInput BeamBreaker1 = new DigitalInput(2);
  // public static DigitalInput ArmBeamBreaker = new DigitalInput(0);
 

  /** Creates a new Intake. */
  public Intake() {
    this.LeftIntake = new TalonFX(9);
    this.LeftIntake.configFactoryDefault();
    this.LeftIntake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    this.LeftIntake.setSensorPhase(true);
    this.LeftIntake.setInverted(true);
    this.LeftIntake.configPeakOutputForward(1,100);
    this.LeftIntake.configPeakOutputReverse(-1,100);
    this.LeftIntake.config_kF(0, 0.0,100);
    this.LeftIntake.config_kP(0, 0,100);
    this.LeftIntake.config_kI(0, 0,100);
    this.LeftIntake.config_kD(0, 0,100);
    this.LeftIntake.setNeutralMode(NeutralMode.Coast);
    

    this.RightIntake = new TalonFX(8);
    this.RightIntake.configFactoryDefault();
    this.RightIntake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    this.RightIntake.setSensorPhase(true);
    this.RightIntake.setInverted(false);
    this.RightIntake.configPeakOutputForward(1,100);
    this.RightIntake.configPeakOutputReverse(-1,100);
    this.RightIntake.config_kF(0, 0.0,100);
    this.RightIntake.config_kP(0, 0,100);
    this.RightIntake.config_kI(0, 0,100);
    this.RightIntake.config_kD(0, 0,100);
    this.RightIntake.setNeutralMode(NeutralMode.Coast);

    this.Intake = new TalonFX(10,"Canivore");
    this.Intake.configFactoryDefault();
    this.Intake.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    this.Intake.setSensorPhase(true);
    this.Intake.setInverted(false);
    this.Intake.configNominalOutputForward(0);
    this.Intake.configNominalOutputReverse(0);
    this.Intake.configPeakOutputForward(1);
    this.Intake.configPeakOutputReverse(-1);
    this.Intake.configAllowableClosedloopError(0, 0, 30);
    this.Intake.config_kF(0, 0);
    this.Intake.config_kP(0, 0.05);
    this.Intake.config_kI(0, 0);
    this.Intake.config_kD(0, 0);
    this.Intake.setNeutralMode(NeutralMode.Coast);
    this.Intake.configClosedloopRamp(CubeSpeed);
    this.Intake.configOpenloopRamp(CubeSpeed);
    this.Intake.selectProfileSlot(0, 0);
    this.Intake.configMotionCruiseVelocity(100000, 30);
		this.Intake.configMotionAcceleration(100000, 30);

    this.LeftIntake.follow(this.RightIntake);

    this.Intake.setSelectedSensorPosition(0);
 
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left Motor power", this.LeftIntake.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Motor power", this.RightIntake.getMotorOutputPercent());
    SmartDashboard.putNumber("Intake Motor power", this.Intake.getMotorOutputPercent());
    SmartDashboard.putNumber("Intake Motor Current", this.Intake.getStatorCurrent());
    SmartDashboard.putNumber("Intake Velocitty",this.Intake.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Intake Posittion", this.Intake.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Bintake", detectBeamBreaker1());
    SmartDashboard.putNumber("Cube Position",CubeSpeed);
    SmartDashboard.putNumber("Cube Degrees",Cubedegrees);
    SmartDashboard.putString("Output", "Test");
    SmartDashboard.putNumber("Intake Error", this.Intake.getSelectedSensorPosition()-((Cubedegrees*(((4*4*4*2.9*2.9)*2048)/360))));

    SmartDashboard.putNumber("Checking the cube degrees",PositionIntake());
    // SmartDashboard.putBoolean("Arm Beambreaker", ArmBeamBreaked());
    // SmartDashboard.putNumber("")
    // if(this.RightIntake.isRevLimitSwitchClosed()==1){
    //   this.LeftIntake.set(TalonFXControlMode.PercentOutput,0);
    //   this.RightIntake.set(TalonFXControlMode.PercentOutput,0);

    // }

    // This method will be called once per scheduler run
  }
  // public void ()
  public void IntakeCube(double velocity) {
  // if(detectBeamBreaker1()==false){
  
  this.RightIntake.set(TalonFXControlMode.PercentOutput, -velocity);
  // }
  // else if(detectBeamBreaker1()==true){
  // this.RightIntake.set(TalonFXControlMode.PercentOutput, 0);
  // }
}
  public void OuttakeCube(double velocity){
    this.RightIntake.set(TalonFXControlMode.PercentOutput, -velocity);
  }

  public void Intake_LifterUp(boolean Lifter_up) {
    if (Lifter_up == false)
      return;

    else if (Lifter_up == true)
      pos += 1;

    increment = ((pos / (538.24)) * 2048) * 300;
    this.Intake.set(TalonFXControlMode.Position, increment);
   
  }

  public void LifterDegrees(double degrees){
   this.Intake.set(TalonFXControlMode.Position, (degrees*(((4*4*2.9*2.9)*2048)/360)));
  }

  // public boolean BeamBreaked(){
  //    return Bintake = BeamBreaker1.get();
  // }

//   public boolean ArmBeamBreaked(){
//     return Barm = ArmBeamBreaker.get();
//  }


  public void Intake_LifterDown(boolean Lifter_down) {
    if (Lifter_down == false)
      return;

    else if (Lifter_down == true)
      pos -= 1;

    increment = ((pos / (538.24)) * 2048) * 300;
    this.Intake.set(TalonFXControlMode.Position, increment);
    
    // Intake.set(-0.2);
  }
  public void ThrowPosition(double Position){
    this.Intake.set(TalonFXControlMode.Position,Position);
  }
  public boolean homeSensorCheck(){
    if(this.Intake.isRevLimitSwitchClosed() == 1){
      // this.hanger.setSelectedSensorPosition(0);
      return true;
    }
    else{
      return false;
    }
   }

   public boolean detectBeamBreaker1() { 
    Bintake = BeamBreaker1.get();
    return Bintake;
  }


  public  void ResetIntake(boolean resetflag, boolean pointflag,boolean Start){
    while(resetflag == true){
    SmartDashboard.putBoolean("Can we Start",Start);
    if(this.Intake.isRevLimitSwitchClosed() == 1 && !homeSensorCheck()==false){
      this.Intake.set(TalonFXControlMode.Velocity,3000 );
     }
 else if(this.Intake.isRevLimitSwitchClosed() == 0){    
  this.Intake.set(TalonFXControlMode.Velocity, -3000);
       pointflag=true;
      SmartDashboard.putString("output", "4");
        while(this.Intake.isRevLimitSwitchClosed() == 1 && pointflag == true){
          this.Intake.set(TalonFXControlMode.Velocity, 0);
          this.Intake.setSelectedSensorPosition(0);
          pos=0;
          Start =true;
          SmartDashboard.putString("output", "5");
          SmartDashboard.putBoolean("Can we Start",Start);
          return;
        }
    }
   }
   return;
}



public void OperatorCubeSpeed(double operatorcubespeed){
  CubeSpeed =  operatorcubespeed;
}

public void OperatorCubeDegrees(double operatorcubedegrees){
   Cubedegrees =operatorcubedegrees; 
}

public void DriverCubeSpeed(){
  this.RightIntake.set(TalonFXControlMode.PercentOutput, -CubeSpeed);
}
public void DriverCubeDegrees(){
  this.Intake.set(TalonFXControlMode.MotionMagic, (Cubedegrees*(((4*4*2.9*2.9)*2048)/360)));
}

public double PositionIntake(){
  return this.Intake.getSelectedSensorPosition()/(((4*4*2.9*2.9)*2048)/360);
}
}
