// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  TalonFX Elbow;
  TalonFX Arm;

  double pos, pos1;
  double increment, increment1;
  CANCoder CANCoderArm;
  CANCoder CANCoderElbow;
  double ArmCancoderzero, ElbowCancoderzero;
  double OperatorCancoderArmValues;
  double OperatorCancoderElbowValues;
  double OperatorGripperDegrees;
  boolean heightposition;
  boolean GripperRotate,GripperBB;
  double WristDegrees;
  double CurrentDegrees;

  public static DigitalInput GripperRoatateBeamBreaker = new DigitalInput(0);
  public static DigitalInput GripperBeamBreaker = new DigitalInput(3);

  public CANSparkMax Wrist = new CANSparkMax(15, MotorType.kBrushless);

  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
 
  private double armspeed = 0.3, elbowspeed = 0.3;
  // private double ChangedArmSpeed = 0.3, ChangedElbowSpeed = 0.3;
  private boolean Ground;

  /** Creates a new Elbow. */
  public Arm() {
   

    this.Elbow = new TalonFX(6,"Canivore");
    this.Elbow.configFactoryDefault();
    this.Elbow.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    this.Elbow.setSensorPhase(true);
    this.Elbow.setInverted(true);
    this.Elbow.configNominalOutputForward(0);
    this.Elbow.configNominalOutputReverse(0);
    this.Elbow.configPeakOutputForward(elbowspeed);
    this.Elbow.configPeakOutputReverse(-elbowspeed);
    this.Elbow.configAllowableClosedloopError(0, 0, 30);
    this.Elbow.config_kF(0, 0);
    this.Elbow.config_kP(0, 0.45);
    this.Elbow.config_kI(0, 0);
    this.Elbow.config_kD(0, 0);
    this.Elbow.setNeutralMode(NeutralMode.Coast);
    // this.Elbow.selectProfileSlot(0, 0);
    // this.Elbow.configMotionCruiseVelocity(10000, 30);
		// this.Elbow.configMotionAcceleration(10000, 30);

    this.Arm = new TalonFX(7,"Canivore");
    this.Arm.configFactoryDefault();
    this.Arm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    this.Arm.setSensorPhase(true);
    this.Arm.setInverted(true);
    this.Arm.configNominalOutputForward(0);
    this.Arm.configNominalOutputReverse(0);
    this.Arm.configPeakOutputForward(armspeed);
    this.Arm.configPeakOutputReverse(-armspeed);
    this.Arm.configAllowableClosedloopError(0, 0, 30);
    this.Arm.config_kF(0, 0);
    this.Arm.config_kP(0, 0.45);
    this.Arm.config_kI(0, 0);
    this.Arm.config_kD(0, 0);
    this.Arm.setNeutralMode(NeutralMode.Coast);
    // this.Arm.selectProfileSlot(0, 0);
    // this.Arm.configMotionCruiseVelocity(10000, 30);
		// this.Arm.configMotionAcceleration(10000, 30);
    this.CANCoderArm = new CANCoder(9,"Canivore");
    this.CANCoderElbow = new CANCoder(7,"Canivore");
    CANCoderArm.configFactoryDefault();
    CANCoderElbow.configFactoryDefault();

    CANCoderArm.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    CANCoderElbow.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    
    Wrist.setInverted(true);
    Wrist.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    m_pidController = Wrist.getPIDController();
    m_pidController.setOutputRange(-0.5, 0.5);
    m_encoder = Wrist.getEncoder(Type.kHallSensor,42);
    m_pidController.setP(0.5);

    ArmCancoderzero = 222.36;
    ElbowCancoderzero = 220;
    this.Arm.setSelectedSensorPosition((ArmCancoderzero - CANCoderArm.getAbsolutePosition()) * 1706.67);
    this.Elbow.setSelectedSensorPosition((ElbowCancoderzero - CANCoderElbow.getAbsolutePosition()) * 1706.67);
    
    //HomePostion();
    // this.Arm.set(ControlMode.Position, this.Arm.getSelectedSensorPosition());
    // this.Elbow.set(ControlMode.Position, this.Arm.getSelectedSensorPosition());
    // this.Arm.set(TalonFXControlMode.Position, 0);
    // this.Elbow.set(TalonFXControlMode.Position, 0);

    // intake = new Intake();

    // this.Arm.setSelectedSensorPosition(0);
    // this.Elbow.setSelectedSensorPosition(00);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Cancoder Arm", CANCoderArm.getAbsolutePosition());
    SmartDashboard.putNumber("Cancoder Elbow", CANCoderElbow.getAbsolutePosition());
    SmartDashboard.putBoolean("height mode", heightposition);
    SmartDashboard.putNumber("Arm Sensor Position", this.Arm.getSelectedSensorPosition());

    SmartDashboard.putNumber("Arm Power", this.Arm.getMotorOutputPercent());
    SmartDashboard.putNumber("Elbow Power", this.Elbow.getMotorOutputPercent());
    SmartDashboard.putBoolean("GripperRotateBeamBreaked", GripperRotateBeamBreaked());
    SmartDashboard.putBoolean("GripperBeamBreaked", GripperBeamBreaked());
    SmartDashboard.putNumber("Encoder Value",m_encoder.getPosition());
    SmartDashboard.putNumber("Degrees of Intake",WristDegrees);
    SmartDashboard.putNumber("Current Degrees of Intake",CurrentDegrees);
    SmartDashboard.putNumber("Arm Velocity", this.Arm.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Elbow Velocity", this.Elbow.getSelectedSensorVelocity());
    // SmartDashboard.putNumber("Arm Current", this.Arm.getStatorCurrent());
    // SmartDashboard.putNumber("Elbow Current", this.Elbow.getStatorCurrent());
    // SmartDashboard.putNumber("Arm Position",
    // this.Arm.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Elbow Position",
    // this.Elbow.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Arm Variable ", pos);
    // SmartDashboard.putNumber("Elbow Variable", pos1);
    // SmartDashboard.putNumber("ERror Arm", increment -
    // this.Arm.getSelectedSensorPosition());
    // SmartDashboard.putNumber("ERror Elbow", increment1 -
    // this.Elbow.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Difference
    // Arm",ArmFalconSensrUnits()-CountsFromArmCancoder());
    // SmartDashboard.putNumber("Difference
    // ELbow",ElbowFalconSensrUnits()-CountsFromElbowCancoder());
    // SmartDashboard.putNumber("Closed loop error: ",
    // this.Arm.getClosedLoopError());
    // SmartDashboard.putNumber("Elbow Degrees", CountsFromElbowCancoder(32 -
    // 90.5));
    // SmartDashboard.putNumber("Arm Degrees", 20 -
    // CANCoderArm.getAbsolutePosition());
    // SmartDashboard.putNumber("OperatorCancoderArmValues",
    // OperatorCancoderArmValues);
    // SmartDashboard.putNumber("OperatorCancoderElboowValues",
    // OperatorCancoderElbowValues);

    // editOutputArmConfig();
    // editOutputElbowConfig();
    // if (m_colorSensor.getProximity() > 300) {
    // // SolGrip.set(true);
    // Stop();
    // }

  }

  public void ManualarmUp(boolean armis_up) {
    if (armis_up == false)
      return;
    else if (armis_up == true)
      pos += 1;
    increment = ((pos / (100 * 4)) * 2048) * 50;
    // if (pos >= 500){
    // pos = 500;
    // }
    this.Arm.set(TalonFXControlMode.Position, increment);
  }

  public void HomePostion() {
    // ArmCancoderzero= 139.80;
    // ElbowCancoderzero = 28.125;
    SetOperatorArmCancoderValues(ArmCancoderzero);
    SetOperatorELbowCancoderValues(ElbowCancoderzero);
    this.Elbow.set(TalonFXControlMode.Position,  CountsFromElbowCancoder());
    this.Arm.set(TalonFXControlMode.Position,  CountsFromArmCancoder());
    

  }

  // public void IntakePosition() {
  //   this.Arm.set(TalonFXControlMode.Position, CountsFromArmCancoder());
  //   this.Elbow.set(TalonFXControlMode.Position, CountsFromElbowCancoder());
  //   this.intake.Intake.set(TalonFXControlMode.Position, 364188 * 0.998);
  // }

  // public void IntakeCOne() {
  //   if (this.intake.Intake.getSelectedSensorPosition() >= 247000) {
  //     this.Arm.set(TalonFXControlMode.Position, CountsFromArmCancoder());
  //     this.Elbow.set(TalonFXControlMode.Position, CountsFromElbowCancoder(26 - 23.46));
  //     this.intake.RightIntake.set(TalonFXControlMode.PercentOutput, 0.08);
  //     Timer.delay(1.7);
  //     this.Arm.set(TalonFXControlMode.Position, CountsFromArmCancoder());
  //     this.Elbow.set(TalonFXControlMode.Position, CountsFromElbowCancoder(26 - 43.75));
  //     this.intake.RightIntake.set(TalonFXControlMode.PercentOutput, 0);
  //     Timer.delay(1.2);
  //     this.Arm.set(TalonFXControlMode.Position, CountsFromArmCancoder());
  //     this.Elbow.set(TalonFXControlMode.Position, CountsFromElbowCancoder(26 - 44.47));
  //     Timer.delay(2);
  //     this.Arm.set(TalonFXControlMode.Position, 0);
  //     this.Elbow.set(TalonFXControlMode.Position, 0);
  //   } else {
  //     return;
  //   }
  // }

  // public void MediumPosition() {
  //   this.Arm.set(TalonFXControlMode.Position, CountsFromArmCancoder());
  //   this.Elbow.set(TalonFXControlMode.Position, CountsFromElbowCancoder(26 - 36.24));
  //   this.intake.Intake.set(TalonFXControlMode.Position, 247000);
  //   Timer.delay(3);
  //   this.Arm.set(TalonFXControlMode.Position, CountsFromArmCancoder());
  //   this.Elbow.set(TalonFXControlMode.Position, CountsFromElbowCancoder(26 - 79.24));
  // }

  // public void HighPosition() {
  //   this.Elbow.set(TalonFXControlMode.Position, CountsFromElbowCancoder(32 - 112.5));
  //   Timer.delay(0.4);
  //   this.Arm.set(TalonFXControlMode.Position, CountsFromArmCancoder());
  // }

  public void ManualElbowUp(boolean Elbowis_up) {
    if (Elbowis_up == false)
      return;
    else if (Elbowis_up == true)
      pos1 += 1;
    increment1 = ((pos1 / (100 * 4)) * 2048) * 200;
    // if (pos >= 500){
    // pos = 500;
    // }
    this.Elbow.set(TalonFXControlMode.Position, increment1);
  }

  public void ManualarmDown(boolean armis_down) {
    if (armis_down == false)
      return;
    else if (armis_down == true)
      pos -= 1;
    increment = ((pos / (100 * 4)) * 2048) * 50;
    // if (pos <= 30){
    // pos = 30;
    // }
    this.Arm.set(TalonFXControlMode.Position, increment);
  }

  public void ManualElbowDown(boolean Elbowis_down) {
    if (Elbowis_down == false)
      return;
    else if (Elbowis_down == true)
      pos1 -= 1;
    increment1 = ((pos1 / (100 * 4)) * 2048) * 200;
    // if (pos1 <= 30){
    // pos1 = 30;
    // }
    this.Elbow.set(TalonFXControlMode.Position, increment1);
  }

  public void MoveArm(double degrees) {
    this.Arm.set(TalonFXControlMode.Position, this.Arm.getSelectedSensorPosition() + CountsFromArmDegree(degrees));
  }

  public void MoveElbow(double degrees) {
    this.Elbow.set(TalonFXControlMode.Position,
        this.Elbow.getSelectedSensorPosition() + CountsFromElbowDegree(degrees));
  }

  public double getCancoderArmValue() {
    return CANCoderArm.getAbsolutePosition();
  }

  public double getCancoderElbowValue() {
    return CANCoderElbow.getAbsolutePosition();
  }

  // public void Intake(double grippingspeed) {
  // Intake.set(grippingspeed);
  // }

  // public void Stop() {
  // m_pidController.setReference(m_encoder.getPosition(), ControlType.kPosition);
  // }

  public double angleWrap(double degrees) {
    while (degrees > 180) {
      degrees -= 360;
    }
    while (degrees < -180) {
      degrees += 360;
    }
    return degrees;
  }

  public double CancoderElbowAngleWrap() {
    return angleWrap(CANCoderElbow.getAbsolutePosition());
  }

  public double CountsFromArmDegree(double ArmDegree) {
    return ArmDegree * 568.89;
  }

  public double ArmDegreefromCancoder(double ArmCancoderValue) {
    return ArmCancoderValue * 9;
  }

  public double CancoderfromArmDegree(double ArmDegree) {
    return ArmDegree / 9;
  }

  public double CountsFromElbowCancoder(double CancoderElbowValue) {
    return CancoderElbowValue * 1706.67;
  }

  public double CountsFromElbowDegree(double ElbowDegree) {
    return ElbowDegree * 568.89;
  }

  public double ELbowDegreefromCancoder(double ElbowCancoderValue) {
    return ElbowCancoderValue * 9;
  }

  public double CancoderfromELbowDegree(double ELbowDegree) {
    return ELbowDegree / 9;
  }

  public void setHeight(boolean height) {
    if (height == true) {
      heightposition = true;
      SmartDashboard.putString("heightposition", "true");
    } else if (height == false) {
      heightposition = false;     
       SmartDashboard.putString("heightposition", "false");
    }
  }

  public boolean heightType() {
    return heightposition;
  }

  // public void setArmspeed(double Armspeed) {
  //   ChangedArmSpeed = Armspeed;
  // }

  // public void editOutputArmConfig() {
  //   double ArmSpeed = ChangedArmSpeed;
  //   if (armspeed != ArmSpeed) {
  //     armspeed = ArmSpeed;
  //     this.Arm.configPeakOutputForward(armspeed);
  //     this.Arm.configPeakOutputReverse(-armspeed);
  //     SmartDashboard.putNumber("ArmSpeedddddd", ArmSpeed);
  //   }
  // }

  // public void setElbowspeed(double Elbowspeed) {
  //   ChangedElbowSpeed = Elbowspeed;
  // }

  // public void editOutputElbowConfig() {
  //   double ElbowSpeed = ChangedElbowSpeed;
  //   if (elbowspeed != ElbowSpeed) {
  //     elbowspeed = ElbowSpeed;
  //     this.Elbow.configPeakOutputForward(elbowspeed);
  //     this.Elbow.configPeakOutputReverse(-elbowspeed);
  //   }
  // }

  public void SetOperatorArmCancoderValues(double setoperatorarmcancodervalues) {
    OperatorCancoderArmValues = setoperatorarmcancodervalues;
  }

  public double CountsFromArmCancoder() {
    return ((ArmCancoderzero - OperatorCancoderArmValues) * 1706.67);
  }

  public void setArmcancoderDegrees() {
    this.Arm.set(TalonFXControlMode.Position, CountsFromArmCancoder());
  }

  public void SetOperatorELbowCancoderValues(double setoperatorcancodervalues) {
    OperatorCancoderElbowValues = setoperatorcancodervalues;
  }

  public double CountsFromElbowCancoder() {
    return ((ElbowCancoderzero - OperatorCancoderElbowValues) * 1706.67);
  }

  public void setElbowcancoderDegrees() {
    this.Elbow.set(TalonFXControlMode.Position, CountsFromElbowCancoder());
  }

  public double ArmFalconSensrUnits() {
    return this.Arm.getSelectedSensorPosition();
  }

  public double ElbowFalconSensrUnits() {
    return this.Elbow.getSelectedSensorPosition();
  }

  public void InputSUBORGRO(boolean Input_type) {
    if (Input_type == true) {
      Ground = true;
      SmartDashboard.putString("InputSUBORGRO", "Ground");
    } else if (Input_type == false) {
      Ground = false;
      SmartDashboard.putString("InputSUBORGRO", "Substation");
    }
  }

  public void GripperDegrees(double gripperdegrees) {
    OperatorGripperDegrees = gripperdegrees;
  }

  public double GripperValueConversion() {
    return OperatorGripperDegrees * (((2.4 * 100) ) / 360);
  }

  public void GripperRotate() {
    this.m_pidController.setReference(GripperValueConversion(), CANSparkMax.ControlType.kPosition);
  }


  public void manualGripperUpOrDown(double degrees) {
    CurrentDegrees = CurrentDegrees+degrees;
    WristDegrees= WristDegrees + ( degrees * ((2.4 * 100) / 360));
    this.m_pidController.setReference(WristDegrees,
        CANSparkMax.ControlType.kPosition);
  }
  

  public boolean GroundType() {
    return Ground;
  }

  public boolean GripperRotateBeamBreaked() {
    GripperRotate = GripperRoatateBeamBreaker.get();
    return GripperRotate;
  }

    public boolean GripperBeamBreaked() {
    GripperBB = GripperBeamBreaker.get();
    return GripperBB;
  }

  public  void ResetGripper(boolean resetflag, boolean pointflag,boolean Start){
    while(resetflag == true){
    SmartDashboard.putBoolean("Can we Start Grippper",Start);
    if(GripperRotateBeamBreaked()== false){
      this.Wrist.set(0.05);
     }
  else if(GripperRotateBeamBreaked() == true){    
    this.Wrist.set(-0.05);
       pointflag=true;
      SmartDashboard.putString("output", "4");
        while(GripperRotateBeamBreaked() == false && pointflag == true){
          this.Wrist.set(0);
          m_encoder.setPosition(0);
          GripperDegrees(-6);
          GripperRotate();
          Start =true;
          SmartDashboard.putString("output", "5");
          SmartDashboard.putBoolean("Can we Start Grippper",Start);
          return;
        }    
    }
   }
   return;
  }


}
