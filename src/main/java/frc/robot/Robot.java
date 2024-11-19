// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;




import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.DriverJoysticks;
import frc.robot.Commands.OperatorJoysticks;
import frc.robot.Subsystems.SwerveModule;
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
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
 
  SwerveModule[] LowRide = {
     //Front Left
    new SwerveModule(
      2, //low 2 || 
    1, // Low 1 ||
    true, // Low true ||
    false, // False ||
    9, // Low 9 ||
    0, // Low 0 ||
    true),
    //Front Right
    new SwerveModule(
      4, //Low 4 ||
      3, // Low 3 ||
      false, // Low False
      false, // Low false
      10, // low 10 ||
      0, // Low 0 ||
      true),
      //Back Right
    new SwerveModule(
      6, // Low 6 ||
      5, // Low 5 //
      false, //Low False ||
      false,//Low False ||
      11, //Low 11 ||
      0, // Low 0 ||
      true),
      //Back Left
      new SwerveModule(
        8, // Low 8 ||
        7, // Low 7 ||
        true,// Low true ||
        false, //Low False ||
        12, // Low 12 ||
        0, // Low 0 ||
        true) //Low true ||
  };
  SwerveSubsystem DriveBase = new SwerveSubsystem(LowRide,new Pigeon2(22)); 
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
    DriveBase.intialize();
   // SwerveMove = new SwerveControllerCommand(DriveBase.tr, DriveBase::getPos, Constants.kDriveKinematics,DriveBase.Holo, output -> DriveBase.setModuleState(output), DriveBase);
   // SwerveMove.initialize();
  }

  /** This function is called periodically during autonomous. */
   // Create config for trajectory
 


 // Command SwerveMove = new SwerveControllerCommand(exampleTrajectory, DriveBase::getPos, Constants.kDriveKinematics,DriveBase.Holo, output -> DriveBase.setModuleState(output), DriveBase);

  @Override
  public void autonomousPeriodic() {
    //  SwerveMove.execute();
      System.out.println(DriveBase.getPos());
      
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
 // DriveStick.execute();
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
