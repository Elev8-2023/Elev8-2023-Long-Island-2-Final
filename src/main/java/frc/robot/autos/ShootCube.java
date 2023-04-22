// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ShootCube extends CommandBase {
  Intake intake;
  double pow;
  boolean shootflag;
  double lastTimeStamp,timer=0;
  /** Creates a new ShootCube. */
  public ShootCube(Intake intake, double pow) {
    this.intake = intake;
    this.pow = pow;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootflag = false;
    timer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.lastTimeStamp = (Timer.getFPGATimestamp()-this.timer);
    // cubeSubsystem.intakeCubeRPM(RPM);
    this.intake.OperatorCubeSpeed(-pow);
    this.intake.DriverCubeSpeed();
    lastTimeStamp = (Timer.getFPGATimestamp()-timer);
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(this.lastTimeStamp>=0.5){
      this.intake.OperatorCubeSpeed(0);
      this.intake.DriverCubeSpeed();
      this.intake.OperatorCubeDegrees(20);
      this.intake.DriverCubeDegrees();
      return true;
      // cubeSubsystem.intakeCubeRPM(0);
    }
    return false;
  }
  
}