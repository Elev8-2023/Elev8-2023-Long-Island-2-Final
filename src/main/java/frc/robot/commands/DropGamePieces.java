package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.ElementSelector;;

public class DropGamePieces extends CommandBase {
  private Intake intake;
  private ElementSelector mode;
  private double timer, lasttimestamp = 0;
  private Gripper grip;
  private Arm arm;
  private boolean returnflag;
  /** Creates a new MediumPosition. */
  public DropGamePieces(Intake intake, ElementSelector mode,Arm arm,Gripper grip) {
    this.intake = intake;
    this.mode = mode;
    this.grip=grip;
    this.arm=arm;
    addRequirements(intake,mode);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer = Timer.getFPGATimestamp();
    returnflag=false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.mode.Mode==true){
      this.intake.DriverCubeSpeed();

      SmartDashboard.putString("OutputPower", "Cube Speed");
      // this.lasttimestamp = (Timer.getFPGATimestamp() - this.timer);
    }
    else if(this.mode.Mode == false){

      this.grip.IsGrip(true);
      this.grip.Grip();
      Timer.delay(0.25);
      this.arm.GripperDegrees(0);
      this.arm.GripperRotate();
      this.arm.HomePostion();
    }
    this.lasttimestamp = (Timer.getFPGATimestamp  () - this.timer);
      
    if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
    && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200)) {
      // this.intake.OuttakeCube(0);
      this.grip.IsGrip(false);
      this.grip.Grip();
      
      returnflag=true;
} 
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished(){
    if(this.mode.Mode==true){
    if (this.lasttimestamp > 1){
    // Timer.delay(0.5);

    this.intake.OuttakeCube(0);
    this.intake.OperatorCubeDegrees(0);
    this.intake.DriverCubeDegrees();
    return true;
  }
  else {
    return false;
  }
  }
  if(this.mode.Mode == false){ 
    if(returnflag == true){
    this.intake.OperatorCubeDegrees(0);
    this.intake.DriverCubeDegrees();
    return true;
  }
  else {
    return false;
  }
}
else return false;
}}
