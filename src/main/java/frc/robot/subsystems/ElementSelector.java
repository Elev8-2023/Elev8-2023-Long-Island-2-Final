package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElementSelector extends SubsystemBase {
  public boolean Mode;
  public static DigitalOutput ledR;
  public static DigitalOutput ledG;
  public static DigitalOutput ledB;

  public ElementSelector() {
    Mode=true;
    ledR = new DigitalOutput(6);
    ledG = new DigitalOutput(7);
    ledB = new DigitalOutput(8);
  }

  @Override
  public void periodic() {
    if(!Mode){
      ledG.set(false);
      ledB.set(true);
      ledR.set(true);
    }
    else{
      ledG.set(true);
      ledB.set(false);
      ledR.set(true);
    }
  }

  public void mode(boolean InputType) {
    if(InputType == false) { Mode = false; SmartDashboard.putString("Input type", "Cone"); } 
    else if(InputType == true) { Mode = true; SmartDashboard.putString("Input type", "Cube"); } 
  }

  public boolean Type() {
    return Mode;
  }
}
