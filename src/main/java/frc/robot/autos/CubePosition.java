// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class CubePosition extends CommandBase {
  private Intake intake;
  double angle;
  boolean flag;
  /** Creates a new CubePosition. */
  public CubePosition(Intake intake, double angle) {
    this.intake = intake;
    this.angle = angle;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.intake.OperatorCubeDegrees(angle);
    this.intake.DriverCubeDegrees();
    if (intake.getIntakePosition()>=(0.95*angle) && intake.getIntakePosition()<=(1.05*angle)) {
      flag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flag;
  }
}
