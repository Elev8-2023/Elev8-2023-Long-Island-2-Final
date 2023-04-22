package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualGripper extends CommandBase {
  /** Creates a new ManualGripper. */
  Arm arm;
  double degrees;
  boolean flag;
  public ManualGripper(Arm arm, double degrees) {
    this.arm = arm;
    this.degrees = degrees;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    flag = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.GripperDegrees(degrees); //110, 140
    arm.GripperRotate();
    flag = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flag;
  }
}
