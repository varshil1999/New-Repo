package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.autos.IntakeCube;
import frc.robot.autos.SetIntakeAngle;
import frc.robot.autos.ShootCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.InputMode;
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

public class ConeAuto extends SequentialCommandGroup {
  

    public ConeAuto(Swerve s_Swerve,Intake intake,Arm arm,Gripper gripper) {

        HashMap<String, Command> eventMap = new HashMap<>();
        
        List<PathPlannerTrajectory> pathgroup1 = PathPlanner.loadPathGroup("1 Cone + Dock", new PathConstraints(3, 3));
        eventMap.put("Place Cone", new Retractarm(intake, arm, gripper));

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
                true,
                s_Swerve // The drive subsystem. Used to properly set the requirements of path following
                         // commands
        );

        Command fullAuto1 = autoBuilder.fullAuto(pathgroup1.get(0));
        // Command fullAuto1 = autoBuilder.fullAuto(pathgroup2.get(1));
        // Command fullAuto2 = autoBuilder.fullAuto(pathgroup2.get(2));
        // Command fullAuto3 = autoBuilder.fullAuto(pathgroup2.get(3));
        // Command fullAuto4 = autoBuilder.fullAuto(pathgroup2.get(4));
        // Command fullAuto5 = autoBuilder.fullAuto(pathgroup2.get(5));
        // Command fullAuto6 = autoBuilder.fullAuto(pathgroup2.get(6));

        // Command fullAuto1 = autoBuilder.fullAuto(pathgroup2.get(0));

        // Command fullAuto1 = autoBuilder.fullAuto(pathgroup1.get(1));

        addCommands(
        
        new InstantCommand(() -> s_Swerve.resetOdometry(pathgroup1.get(0).getInitialPose())),
        new HighConeAuto(intake, arm, gripper),
        fullAuto1,
        new DockBalanceRest(s_Swerve)
        // fullAuto1
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