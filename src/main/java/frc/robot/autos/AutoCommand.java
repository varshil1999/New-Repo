package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.autos.IntakeCube;
import frc.robot.autos.SetIntakeAngle;
import frc.robot.autos.ShootCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommand extends SequentialCommandGroup {

    public AutoCommand(Swerve s_Swerve,Intake intake,Arm arm,Gripper gripper) {

        HashMap<String, Command> eventMap = new HashMap<>();
        
        // List<PathPlannerTrajectory> pathgroup1 = PathPlanner.loadPathGroup("5meter straight", new PathConstraints(1, 1));

        // eventMap.put("Lift Low", new ShootCube(cubeSubsystem, 10000));
        
        // eventMap.put("Shoot Cube", new PrintCommand("ARNAV IS A CHAMAN"));

        // eventMap.put("Shoot Cube", new ShootCube(cubeSubsystem, 10000));

        // List<PathPlannerTrajectory> pathgroup2 = PathPlanner.loadPathGroup("2Cubes+Dock", new PathConstraints(4, 3));
        // eventMap.put("FirstCube", new SequentialCommandGroup(new CubePosition(intake),new ShootCube(intake, 0.43)));
        // eventMap.put("Intake", new IntakeCube(intake, 20));
        // eventMap.put("SetHigh", new CubePosition(intake));
        // eventMap.put("Shoot High", new ShootCube(intake, 0.43));

        List<PathPlannerTrajectory> pathgroup3 = PathPlanner.loadPathGroup("Cone+Cube+Dock", new PathConstraints(3, 2.5));
      
        eventMap.put("RetractArm", new Retractarm(intake, arm, gripper));
        eventMap.put("Intake", new IntakeCube(intake, 20));
        eventMap.put("Cube Position", new CubePosition(intake));
        eventMap.put("Cube Shoot", new ShootCube(intake, 0.43));

        
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
                new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
                                                  // rotation controller)
                s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                false,
                s_Swerve // The drive subsystem. Used to properly set the requirements of path following
                         // commands
        );

        Command fullAuto = autoBuilder.fullAuto(pathgroup3.get(0));

        addCommands(
                new HighConeAuto(intake, arm, gripper),
                fullAuto,
                new SequentialCommandGroup(new DockBalance3(s_Swerve),new DockBalanceRest(s_Swerve))
        );

}
}