// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Commands.DriverJoysticks;
import frc.robot.Commands.OperatorJoysticks;
import frc.robot.Subsystems.SwerveSubsystem;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  SwerveSubsystem DriveBase = new SwerveSubsystem(); 
  DriverJoysticks DriveStick = new DriverJoysticks();
  OperatorJoysticks OpJoy = new OperatorJoysticks();


  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
   }

 
  @Override
  public void robotPeriodic() {
    DriveBase.UpdatePos();
}


  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
     m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
   // DriveBase.intialize();
    System.out.println("Auto selected: " + m_autoSelected);
    DriveBase.intialize();
   // SwerveMove = new SwerveControllerCommand(DriveBase.tr, DriveBase::getPos, Constants.kDriveKinematics,DriveBase.Holo, output -> DriveBase.setModuleState(output), DriveBase);
    SwerveMove.initialize();
  }

  /** This function is called periodically during autonomous. */
   // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
              1, // Constants.kPhysicalMaxSpeedMeterPerSecond,
              0.5) // Constants.PhysicalMaxAcclerationUnitsPerSecond)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);
 Command SwerveMove = new SwerveControllerCommand(exampleTrajectory, DriveBase::getPos, Constants.kDriveKinematics,DriveBase.Holo, output -> DriveBase.setModuleState(output), DriveBase);

  @Override
  public void autonomousPeriodic() {
      SwerveMove.execute();
    // Didn't work  // new RunCommand(() -> new SwerveControllerCommand(exampleTrajectory, DriveBase::getPos, Constants.kDriveKinematics,DriveBase.Holo, output -> DriveBase.setModuleState(output), DriveBase));
        System.out.println(DriveBase.getPos());
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here  
      //  new SwerveControllerCommand(DriveBase.tr, DriveBase::getPos, Constants.kDriveKinematics,DriveBase.Holo, output -> DriveBase.setModuleState(output), DriveBase).schedule();
      // System.out.println(DriveBase.getPos());
        break;
      case kDefaultAuto:
      default:
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    Clock.start();
  }

  /** This function is called periodically during operator control. */
  Timer Clock = new Timer();
  int Count = 0;
  Pose2d[] RecordTrajectory = new Pose2d[20];

  @Override
  public void teleopPeriodic() {

  // DriveBase.setModuleState(DriveStick.getRobotRelativeChasisState());

    DriveBase.setModuleState(DriveStick.getFieldRelativeChasisSpeed(DriveBase.getRotation2d()));
    System.out.println(DriveBase.getRotation2d());


    if (OpJoy.RightStick.getRawButtonPressed(5)) {
      DriveBase.InertiaMeasureUnit.reset();
    }

        if (OpJoy.RightStick.getTriggerPressed()) {
      if (Clock.get() >= 0.1) {
        RecordTrajectory[Count] = DriveBase.getPos();
        Count++;
        Clock.restart();
      }
    }
    if (OpJoy.RightStick.getRawButtonPressed(8)) {
      System.out.println(RecordTrajectory);
    }

  SmartDashboard.putNumber("DriveSpeed",DriveBase.frontLeft.driveMotor.get());
  SmartDashboard.putNumber("DriveDirection",DriveBase.frontLeft.getAbsEncoderValue());

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
