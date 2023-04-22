// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.ElementSelector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakePosition extends CommandBase {
  private Intake intake;
  private ElementSelector mode;
  private boolean intakeflag, Groundflag;
  private Arm arm;
  private Gripper grip;
  double GroundCount = 0;
  boolean flag = false, once = true, Substationonce = true, /* Groundonce = true, */ beambreaker = true,
      returnflag = false,
      Groundreturnflag = false;
  int count = 0;
  double armpos1, elbowpos1, armpos2, elbowpos2;

  /** Creates a new IntakePosition. */
  public IntakePosition(Intake intake, ElementSelector mode, Arm arm, Gripper grip) {
    this.intake = intake;
    this.mode = mode;
    this.arm = arm;
    this.grip = grip;
    addRequirements(intake, mode, arm);
  }

  @Override
  public void initialize() {
    // Groundonce = true;
    intakeflag = true;
    count = 0;
    GroundCount = 1;
    // Groundflag = true;
    returnflag = false;
    Groundreturnflag = false;
    once = true;
    flag = true;
    armpos1 = 218.36;// 165;
    elbowpos1 = 294.67 - 22;// -37;
    armpos2 = 238.36;// 185;
    elbowpos2 = 289.17 - 22;// -35.5;
    this.grip.IsGrip(true);
    this.grip.Grip();
  }

  @Override
  public void execute() {

    if (this.mode.Mode == true) {
      if (intakeflag == true) {
        if (this.intake.detectBeamBreaker1() == false) {
          this.intake.OperatorCubeSpeed(0.27);
          this.intake.OperatorCubeDegrees(116);
          this.intake.DriverCubeDegrees();
          if (this.intake.getIntakePosition() >= 80)
            this.intake.DriverCubeSpeed();
          return;
        } else if (this.intake.detectBeamBreaker1() == true) {
          this.intake.IntakeCube(0);
          this.intake.OperatorCubeDegrees(15);
          this.intake.DriverCubeDegrees();
          intakeflag = false;
          return;
        }
      }
    }

    else if (this.mode.Mode == false) {
      if (GroundCount == 1) {
        this.intake.OperatorCubeDegrees(0);
        this.intake.DriverCubeDegrees();
        this.arm.SetOperatorArmCancoderValues(245.485); // 208.125
        this.arm.SetOperatorELbowCancoderValues(276.38 - 22); // -55.84
        this.arm.setArmcancoderDegrees();
        this.arm.setElbowcancoderDegrees();
        GroundCount = 2;
        flag = false;
      } else if (flag == true && GroundCount == 2) {
        this.arm.GripperDegrees(147);
        this.arm.GripperRotate();
        this.grip.IsGrip(true);
        this.grip.Grip();
        GroundCount = 3;
        SmartDashboard.putString("Check Loop", "4");
      } else if (flag == true && GroundCount == 3) {
        this.arm.SetOperatorArmCancoderValues(229.36); // armpos2
        this.arm.SetOperatorELbowCancoderValues(268.4); // 289.17-22 elbowpos2
        this.arm.setArmcancoderDegrees(); 
        this.arm.setElbowcancoderDegrees();
        flag = false;
        GroundCount = 4;
        SmartDashboard.putString("Check Loop", "5");
      } else if (flag == true && GroundCount == 4) {
        if (this.arm.GripperBeamBreaked() == true) {
          Timer.delay(0.15);
          this.grip.IsGrip(false);
          this.grip.Grip();
          Timer.delay(0.75);
          this.arm.GripperDegrees(0);
          this.arm.GripperRotate();
          GroundCount = 5;
          returnflag= true;
        }
      }}
    //   } else if (GroundCount == 5) {
    //     this.arm.SetOperatorArmCancoderValues(249); // 208.125
    //     this.arm.SetOperatorELbowCancoderValues(282.38); // -17.84
    //     this.arm.setArmcancoderDegrees();
    //     this.arm.setElbowcancoderDegrees();
    //     flag = false;
    //   //   GroundCount = 6;
    //   // } else if (GroundCount == 6 && flag == true) {
    //   //   this.arm.SetOperatorArmCancoderValues(261.485); // 208.125
    //   //   this.arm.SetOperatorELbowCancoderValues(256.38); // some other value // -55.84
    //   //   this.arm.setArmcancoderDegrees();
    //   //   this.arm.setElbowcancoderDegrees();
    //     // flag = false;
     
    //     GroundCount = 7;
    //   } else if (GroundCount == 7 && flag == true) {
    //     // this.arm.SetOperatorArmCancoderValues(222.36); // 169
    //     this.arm.SetOperatorELbowCancoderValues(197.55); // -112.67
    //     // this.arm.setArmcancoderDegrees();
    //     this.arm.setElbowcancoderDegrees();
    //     // flag = true;
    //    GroundCount=8;
    //   } 
    //   else if(GroundCount==8){
    //     if(this.arm.getCancoderElbowValue()<250){
    //       this.arm.SetOperatorArmCancoderValues(222.36); // 169
    //       this.arm.setArmcancoderDegrees();
    //       GroundCount = 9;
    //     }
    //   }
    //   else if (GroundCount == 9 && flag == true) {
    //     this.intake.OperatorCubeDegrees(0);
    //     this.intake.DriverCubeDegrees();
    //     returnflag = true;
    //     GroundCount = 10;
    //   }
    // }

    if ((Math.abs(this.arm.ArmFalconSensrUnits() - this.arm.CountsFromArmCancoder()) <= 200)
        && (Math.abs(this.arm.ElbowFalconSensrUnits() - this.arm.CountsFromElbowCancoder()) <= 200))
      flag = true;
    else
      flag = false;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return returnflag;
  }
}