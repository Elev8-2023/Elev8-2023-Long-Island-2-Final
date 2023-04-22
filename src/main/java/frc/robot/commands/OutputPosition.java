// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.ElementSelector;;

public class OutputPosition extends CommandBase {
  private Intake intake;
  private ElementSelector mode;
  private Arm arm;
  private boolean  Mediumflag;
  private double Mediumcount;
  private boolean returnflag;
  private Gripper grip;
  double armpos1,elbowpos1,armpos2,elbowpos2,gripper;
  // int count=0;
  /** Creates a new MediumPosition. */
  public OutputPosition(Intake intake, ElementSelector mode,Arm arm,Gripper grip) {
    this.intake = intake;
    this.mode = mode;
    this.arm=arm;
    this.grip=grip;
    addRequirements(intake,mode);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Mediumflag=true;
    Mediumcount=0;
    returnflag = false;
    // count=0;
    this.grip.IsGrip(false);
    this.grip.Grip();
  
  
    
   if(this.arm.heightType()==true){ //medium
    armpos1= 218.16; //164.80;
    elbowpos1= 267;//-37;
    armpos2=202; //211.56;//158.20;
    elbowpos2= 259;//298.22;//-34;
    gripper=129 ;
  }
  
    if(this.arm.heightType()==false){ //high 
      armpos1= 221.36;//168;
      elbowpos1= 270; //-32;
      armpos2= 169;//128.47;//186m
      elbowpos2= 301;//-7;//288
      gripper = 127;
  }

  //arm - increase is back/ behind, decrease is ahead
  //elbow - increase is up/ decrease is down

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.mode.Mode==true){
      this.intake.DriverCubeDegrees();
    }

    else if(this.mode.Mode == false){
      if ( Mediumcount < 0.25) {
        this.intake.OperatorCubeSpeed(-0.25);
        this.intake.OperatorCubeDegrees(50);
        this.intake.DriverCubeDegrees();
        Mediumcount=0.28;
        
      }
      if( this.intake.getIntakePosition() >= 35 ){ 
        if (Mediumflag == true && Mediumcount< 1) {
          this.arm.SetOperatorArmCancoderValues(armpos1);
          this.arm.SetOperatorELbowCancoderValues(elbowpos1);
          this.arm.setArmcancoderDegrees();
          this.arm.setElbowcancoderDegrees();
          Mediumcount = 1;
        } else if (Mediumflag == true && Mediumcount == 1) {
          this.arm.SetOperatorArmCancoderValues(armpos2);
          this.arm.SetOperatorELbowCancoderValues(elbowpos2);
          this.arm.GripperDegrees(gripper);
          this.arm.GripperRotate();
          this.arm.setArmcancoderDegrees();
          this.arm.setElbowcancoderDegrees();
          returnflag=true;
        }
        
        

    }
    if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
            && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200)) {
          Mediumflag = true;
        } else {
          Mediumflag = false;
        }
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return returnflag;
  }
}