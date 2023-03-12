// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class InputMode extends SubsystemBase {
  public boolean Mode;
  /** Creates a new Mode. */
  public InputMode() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void mode(boolean Input_type){
    if(Input_type == true){
      Mode = true;
      SmartDashboard.putString("INPUTTYPE", "cube");
    }
    else if(Input_type == false){
      Mode = false;
      SmartDashboard.putString("INPUTTYPE", "Cone");
    }
  }

  public boolean Type(){
    return Mode;
  }
}
