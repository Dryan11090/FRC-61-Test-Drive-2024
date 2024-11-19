package frc.robot.Commands;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

public class AutoCommand {

 SwerveModule DriveBase;

    TrajectoryConfig config =
        new TrajectoryConfig(
              1, // Constants.kPhysicalMaxSpeedMeterPerSecond,
              0.5) // Constants.PhysicalMaxAcclerationUnitsPerSecond)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics);


    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

   // Command SwerveMove = new SwerveControllerCommand(exampleTrajectory, this.DriveBase::getPos, Constants.kDriveKinematics, this.DriveBase.Holo, output -> DriveBase.setModuleState(output), DriveBase);

}
