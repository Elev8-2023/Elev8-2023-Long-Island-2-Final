package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCommand extends SequentialCommandGroup {
    Intake intake = new Intake();
    public AutoCommand(Swerve s_Swerve) {

        HashMap<String, Command> eventMap = new HashMap<>();
        
        List<PathPlannerTrajectory> pathgroup1 = PathPlanner.loadPathGroup("5meter straight", new PathConstraints(1, 1));

        // eventMap.put("Lift Low", new ShootCube(cubeSubsystem, 10000));
        
        // eventMap.put("Shoot Cube", new PrintCommand("ARNAV IS A CHAMAN"));

        // eventMap.put("Shoot Cube", new ShootCube(cubeSubsystem, 10000));

        List<PathPlannerTrajectory> pathgroup2 = PathPlanner.loadPathGroup("Auto 3", new PathConstraints(1, 1));
        eventMap.put("Intake", new IntakeCube(intake, 20));
        eventMap.put("Angle Low", new CubePosition(intake, 43));
        eventMap.put("Shoot Low", new ShootCube(intake, 0.1));
        //eventMap.put("Angle Mid", new InstantCommand(()-> new SequentialCommandGroup(new InstantCommand(()->this.intake.OperatorCubeDegrees(43)),new InstantCommand(()->this.intake.DriverCubeDegrees()))));
        eventMap.put("Angle Mid", new CubePosition(intake, 43));
        eventMap.put("Shoot Mid", new ShootCube(intake, 0.25));
        eventMap.put("Angle High",new SequentialCommandGroup(new SetIntakeAngle(intake),new ShootCube(intake, 0.45)) );
        eventMap.put("Shoot High", new PrintCommand("Done"));

        
        // List<PathPlannerTrajectory> pathgroup2 = PathPlanner.loadPathGroup("Hello World", new PathConstraints(2.0, 2.0));

        
                    // This trajectory can then be passed to a path follower such as a
        // PPSwerveControllerCommand
        // Or the path can be sampled at a given point in time for custom path following

        // Sample the state of the path at 1.2 seconds
        // PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

        // Print the velocity at the sampled time
        // System.out.println(exampleState.velocityMetersPerSecond);
        

        
                // eventMap.put("Intake", new IntakeCube(cubeSubsystem));
        // eventMap.put("Lift Low", new InstantCommand(()->cubeSubsystem.setCubeIntakeAngle(40)));

        // eventMap.put("Shoot Low", new ShootCube(cubeSubsystem, 5000));
        // eventMap.put("Intake", new IntakeCube(cubeSubsystem));
        // eventMap.put("Lift Low", new InstantCommand(()->cubeSubsystem.setCubeIntakeAngle(40)));
       
        // eventMap.put("Intake", new IntakeCube(cubeSubsystem, 80));
        // eventMap.put("Angle Low", new InstantCommand(()-> cubeSubsystem.setCubeIntakeAngle(60)));
        // eventMap.put("Shoot Low", new ShootCube(cubeSubsystem, 5000));
        // eventMap.put("Angle Mid", new InstantCommand(()-> cubeSubsystem.setCubeIntakeAngle(50)));
        // eventMap.put("Shoot Mid", new ShootCube(cubeSubsystem, 5000));
        // eventMap.put("Angle High", new InstantCommand(()-> cubeSubsystem.setCubeIntakeAngle(40)));
        // eventMap.put("Shoot High", new ShootCube(cubeSubsystem, 5000));

        // An example trajectory to follow. All units in meters.

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            
                s_Swerve::getPose, // Pose2d supplier
                s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDConstants(5.8, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
                                                 // and Y PID controllers)
                new PIDConstants(4.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
                                                  // rotation controller)
                s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                false,
                s_Swerve // The drive subsystem. Used to properly set the requirements of path following
                         // commands
        );

        Command fullAuto = autoBuilder.fullAuto(pathgroup1.get(0));
        // Command fullAuto1 = autoBuilder.fullAuto(pathgroup2.get(1));
        // Command fullAuto2 = autoBuilder.fullAuto(pathgroup2.get(2));
        // Command fullAuto3 = autoBuilder.fullAuto(pathgroup2.get(3));
        // Command fullAuto4 = autoBuilder.fullAuto(pathgroup2.get(4));
        // Command fullAuto5 = autoBuilder.fullAuto(pathgroup2.get(5));
        // Command fullAuto6 = autoBuilder.fullAuto(pathgroup2.get(6));

        // Command fullAuto1 = autoBuilder.fullAuto(pathgroup2.get(0));

        // Command fullAuto1 = autoBuilder.fullAuto(pathgroup1.get(1));

        addCommands(
                // new InstantCommand(() -> s_Swerve.resetOdometry(pathgroup2.get(0).getInitialPose())),
                fullAuto
                // fullAuto1//,
                // fullAuto2
                // fullAuto3,
                // fullAuto4,
                // fullAuto5,
                // fullAuto6)
        );
//     }
}
}