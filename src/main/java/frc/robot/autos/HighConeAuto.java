package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.ElementSelector;
import frc.robot.subsystems.Intake;

public class HighConeAuto extends CommandBase {
  private Intake intake;
  private ElementSelector mode;
  private Arm arm;
  private boolean  Mediumflag;
  private double Mediumcount;
  private boolean returnflag;
  private Gripper grip;
  double armpos1,elbowpos1,armpos2,elbowpos2;
  
  public HighConeAuto (Intake intake, Arm arm, Gripper grip) {
    this.intake = intake;
    this.mode = mode;
    this.arm=arm;
    this.grip=grip;
    addRequirements(intake,grip);
  }


  @Override
  public void initialize() {

    this.grip.IsGrip(false);
    this.grip.Grip();
    returnflag =false;
    Mediumflag =true;
    Mediumcount = 0;
    armpos1= 221.36;//168;
      elbowpos1= 265; //-32;
      armpos2= 186.83;//128.47;//186
      elbowpos2= 298;//-7;//288
  }

  @Override
  public void execute() {
    if ( Mediumcount == 0) {
      this.intake.OperatorCubeDegrees(50);
      this.intake.DriverCubeDegrees();
      Mediumcount= 0.25;
     
    }

    if( this.intake.getIntakePosition() >= 45 ){ 
      if (Mediumflag == true && Mediumcount< 1) {

        this.arm.SetOperatorArmCancoderValues(armpos1);
        this.arm.SetOperatorELbowCancoderValues(elbowpos1);
        this.arm.setArmcancoderDegrees();
        this.arm.setElbowcancoderDegrees();
        Mediumcount = 1;
        Mediumflag =false;
        
      } else if (Mediumflag == true && Mediumcount == 1) {
        this.arm.SetOperatorArmCancoderValues(armpos2);
        this.arm.SetOperatorELbowCancoderValues(elbowpos2);
        this.arm.GripperDegrees(140);
        this.arm.GripperRotate();
        this.arm.setArmcancoderDegrees();
        this.arm.setElbowcancoderDegrees();
        Mediumcount = 2;
      } else if(Mediumflag == true && Mediumcount == 2){
        this.grip.IsGrip(true);
        this.grip.Grip();
        returnflag=true;

      }
  }
  if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
          && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200)) {
        Mediumflag = true;
      }
      
      else {
        Mediumflag = false;
      }
}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return returnflag;
  }
}
