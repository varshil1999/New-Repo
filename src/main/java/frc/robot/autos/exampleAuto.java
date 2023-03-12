package frc.robot.autos;

import frc.robot.Constants;
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

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve) {

        PathPlannerTrajectory examplePath = PathPlanner.loadPath("testt", new PathConstraints(2, 2));

        // This trajectory can then be passed to a path follower such as a
        // PPSwerveControllerCommand
        // Or the path can be sampled at a given point in time for custom path following

        // Sample the state of the path at 1.2 seconds
        PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

        // Print the velocity at the sampled time
        System.out.println(exampleState.velocityMetersPerSecond);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("event 1", new PrintCommand("Passed marker 1"));
        eventMap.put("event 2", new WaitCommand(2));
        eventMap.put("event 3", new PrintCommand("Passed marker 3"));

        // An example trajectory to follow. All units in meters.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                s_Swerve::getPose, // Pose2d supplier
                s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDConstants(5.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
                                                 // and Y PID controllers)
                new PIDConstants(5.83, 0.0, 0.0), // PID constants to correct for rotation error (used to create the
                                                  // rotation controller)
                s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                false,
                s_Swerve // The drive subsystem. Used to properly set the requirements of path following
                         // commands
        );

        Command fullAuto = autoBuilder.fullAuto(examplePath);
        addCommands(
                new InstantCommand(() -> s_Swerve.resetOdometry(examplePath.getInitialPose())),
                fullAuto);
    }
}