package frc.robot.Subsystems;


import java.nio.file.Path;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class SwerveSubsystem extends SubsystemBase{

    public static String trajectoryJson = "paths/output/Bounce.wpilib.json";
    public Trajectory tr = new Trajectory();


    public final SwerveModule frontLeft; /*  new SwerveModule
    (
    2, //low 2 || 
    1, // Low 1 ||
    true, // Low true ||
    false, // False ||
    9, // Low 9 ||
    0, // Low 0 ||
    true); // Low true ||
*/
 public final SwerveModule frontRight; /*  new SwerveModule
    (
    4, //Low 4 ||
    3, // Low 3 ||
    false, // Low False
    false, // Low false
    10, // low 10 ||
    0, // Low 0 ||
    true); // Low True
*/
public final SwerveModule backRight; /* new SwerveModule
    (
    6, // Low 6 ||
    5, // Low 5 //
    false, //Low False ||
    false,//Low False ||
    11, //Low 11 ||
    0, // Low 0 ||
    true); // Low true ||
*/
 public final SwerveModule backLeft; /*  new SwerveModule
    (
    8, // Low 8 ||
    7, // Low 7 ||
    true,// Low true ||
    false, //Low False ||
    12, // Low 12 ||
    0, // Low 0 ||
    true); //Low true ||
*/
public double xPos = 0;
public double yPos = 0;
public double FowardOffset = 0;

 private PIDController DrivingXPID = new PIDController(Constants.DrivingProportionalGain, Constants.DrivingReset, Constants.DrivingReset);
 private PIDController DrivingYPID = new PIDController(Constants.DrivingProportionalGain, Constants.DrivingReset, Constants.DrivingReset);
 private ProfiledPIDController AnglePID = new ProfiledPIDController(Constants.DrivingProportionalGain, Constants.DrivingReset, Constants.DrivingReset, 
            new TrapezoidProfile.Constraints(
                4/10*Math.PI, //Max Angular Speed (in Rad/s)
                Math.PI/4) // Max Angular Acceleration (Rad/s^2)
        );
 // private PIDController AnglePID = new PIDController(Constants.AngleProportionalGain, Constants.AngleReset, Constants.AngleReset);
 public HolonomicDriveController Holo = new HolonomicDriveController(DrivingXPID, DrivingYPID, AnglePID);

public Pigeon2 InertiaMeasureUnit; // new Pigeon2(22); //Low 22 || High 35
public final SwerveDriveOdometry odometer; /* = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0),new SwerveModulePosition[] {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backRight.getPosition(),
    backLeft.getPosition()
}); */

    public SwerveSubsystem(SwerveModule[] modules, Pigeon2 pigeon) {
        this.frontLeft = modules[0];
        this.frontRight = modules[1];
        this.backRight = modules[2];
        this.backLeft = modules[3];
        this.InertiaMeasureUnit = pigeon;
        this.odometer = 
        new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0),new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()
        });
    AnglePID.enableContinuousInput(-Math.PI, Math.PI);
    }

    
   public void intialize() {
     AnglePID.enableContinuousInput(-Math.PI, Math.PI);
    resetOdometry(new Pose2d());
        
   try{
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
    tr = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    
        } catch(Exception exc) {DriverStation.reportError("Unable to open trajectory: " + trajectoryJson, exc.getStackTrace());} 
    }

public void UpdatePos() {
 
odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backRight.getPosition(), backLeft.getPosition()}
      );
    SmartDashboard.putNumber("Robot Heading", getHeading());
}
 
//Brandon Spiller, best mechanical design officer and build team lead every. If this line of code is removed, the robot will kill itself
//"im so funny im brandon spiller" - Soumith Madadi (most mid officer ever, maybe less mid than Ben tho)

        
    

    
//Brandon Spiller, best mechanical design officer and build team lead every. If this line of code is removed, the robot will kill itself
//"im so funny im brandon spiller" - Soumith Madadi (most mid officer ever, maybe less mid than Ben tho)

    
    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
        backLeft.resetEncoders();
    }
public void zeroHeading() {
   InertiaMeasureUnit.reset();
}

public double getHeading() {
    return Math.IEEEremainder((InertiaMeasureUnit.getAngle()-FowardOffset) * Math.PI/180, 360); // Get rid of neg for low Add neg for high
}

public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(getHeading());
}

public Pose2d getPos() {
    return odometer.getPoseMeters();
}

public void resetOdometry(Pose2d pose) {
odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backRight.getPosition(), backLeft.getPosition()} ,pose);
}
public void updateOdometer() {
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backRight.getPosition(), backLeft.getPosition()}
      );
}

public void halt() {
        frontLeft.halt();
        frontRight.halt();
        backRight.halt();
        backLeft.halt();
}

public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backRight.stop();
    backLeft.stop();
}
//They are all scrambled up, but this is the correct order of the states list
 public void setModuleState(SwerveModuleState[] requestedState) {
    frontLeft.setState(requestedState[0]);
   frontRight.setState(requestedState[1]);
    backLeft.setState(requestedState[2]);
    backRight.setState(requestedState[3]);

 }
}
