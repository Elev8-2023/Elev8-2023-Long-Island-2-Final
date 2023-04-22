// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ResetCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ResetIntake extends CommandBase {
  private Intake intake;
  boolean pointflag=false, returnflag = false, intakeFlag = false;
  double lastTimeStamp,timer=0;
  /** Creates a new ResetIntake. */
  public ResetIntake(Intake intake) {
    this.intake= intake;

    addRequirements( intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pointflag = false;
    returnflag = false;
    intakeFlag = true;
    timer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeFlag == true && lastTimeStamp==5 )
    lastTimeStamp = (Timer.getFPGATimestamp()-timer);
    if(this.intake.homeSensorCheck() == true){
      this.intake.VelocityControl(3000);
      // SmartDashboard.putNumber("Loop ", 1);
    }
    else if(this.intake.homeSensorCheck()==false){
      this.intake.VelocityControl(-3000);
      pointflag = true;
    }
    if(this.intake.homeSensorCheck() == true && pointflag ==true){
      this.intake.VelocityControl(0);
      returnflag = true;
      intakeFlag = false;
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