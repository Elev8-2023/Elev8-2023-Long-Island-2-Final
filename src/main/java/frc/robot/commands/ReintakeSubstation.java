// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;

public class ReintakeSubstation extends CommandBase {
  private Arm arm;
  private Intake intake;
  private Gripper grip;

  int GroundCount = 0;
  boolean flag, returnflag, GroundOnce = false;

  /** Creates a new ReintakeSubstation. */
  public ReintakeSubstation(Arm arm, Intake intake, Gripper grip) {
    this.arm = arm;
    this.intake = intake;
    this.grip = grip;
    addRequirements(arm, intake, grip);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flag = false;
    returnflag = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.intake.OperatorCubeDegrees(40);
    this.intake.DriverCubeDegrees();
    Timer.delay(0.15);
    this.grip.IsGrip(false);
    this.grip.Grip();
    Timer.delay(0.25);
    this.arm.GripperDegrees(0);
    this.arm.GripperRotate();
    this.arm.HomePostion();
      
    if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
    && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200))
    returnflag = true;
    else
    returnflag = false;

}

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  if(returnflag == true){
    this.intake.OperatorCubeDegrees(0);
    this.intake.DriverCubeDegrees();
    return true;
  }
  else{
    return false;
  }
  }
}