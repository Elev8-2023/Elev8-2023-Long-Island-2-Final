package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  Solenoid SolGrip;
  Compressor comp;
  boolean gripped;
  /** Creates a new Gripper. */
  public Gripper() {
    SolGrip = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    comp = new Compressor(PneumaticsModuleType.CTREPCM);
    // Intake = new CANSparkMax(2, MotorType.kBrushless);
    comp.enableDigital();
    SolGrip.set(false);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("is Gripped", gripped);
  }
  public void IsGrip(Boolean isGripped){
    gripped = isGripped; 
  }

  public void Grip() {
    SolGrip.set(gripped);
  }
}
