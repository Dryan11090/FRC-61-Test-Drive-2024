package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//TODO Fix All of the odomitry related things
//Deleted A bit of unused code
//Commented off update odo due to the methiod using a broken part of the swerveModule class
//Commented off PID for the DriveBase they will be used for transfering Postion to speed for autonomous
public class SwerveSubsystem extends SubsystemBase{
    public final SwerveModule frontLeft = new SwerveModule
    (
    2,
    1,
    true,
    false,
    9,
    0,
    false);

 public final SwerveModule frontRight = new SwerveModule
    (
    4,
    3,
    false,
    false,
    10,
    0,
    true);

public final SwerveModule backLeft = new SwerveModule
    (
    6,
    5,
    false,
    false,
    11,
    0,
    true);

 public final SwerveModule backRight = new SwerveModule
    (
    8,
    7,
    true,
    false,
    12,
    0,
    false);

public double xPos = 0;
public double yPos = 0;

// private PIDController DrivingXPID = new PIDController(Constants.DrivingProportionalGain, Constants.DrivingReset, Constants.DrivingReset);
// private PIDController DrivingYPID = new PIDController(Constants.DrivingProportionalGain, Constants.DrivingReset, Constants.DrivingReset);
// private PIDController AnglePID = new PIDController(Constants.AngleProportionalGain, Constants.AngleReset, Constants.AngleReset);
private Pigeon2 InertiaMeasureUnit = new Pigeon2(35);
public final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0),new SwerveModulePosition[] {
    frontLeft.getPosition(),
    frontRight.getPosition(),
    backLeft.getPosition(),
    backRight.getPosition()
});

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
             //  UpdatePostionReadings();
            } catch (Exception e) {
            }
        }).start();
    }

    public void intialize() {
    // AnglePID.enableContinuousInput(180, -180);
    resetOdometry(new Pose2d());
    }

//New update Odo Collected from Others code TODO Change to make it work properly
public void UpdatePos() {
 
odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backLeft.getPosition(), backRight.getPosition()}
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
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
public void zeroHeading() {
   InertiaMeasureUnit.reset();
}

public double getHeading() {
    return Math.IEEEremainder(InertiaMeasureUnit.getAngle()*Math.PI/180, 360);
}

public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
}

public Pose2d getPos() {
    return odometer.getPoseMeters();
}

public void resetOdometry(Pose2d pose) {
odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backLeft.getPosition(), backRight.getPosition()} ,pose);
}
public void updateOdometer() {
    odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backLeft.getPosition(), backRight.getPosition()}
      );
}

public void halt() {
        frontLeft.halt();
        frontRight.halt();
        backLeft.halt();
        backRight.halt();
}


//This was code used by someone else, Commented out plan to remove after replacing
/* 
@Override
public void periodic() {
 
odometer.update(getRotation2d(), new SwerveModulePosition[] {
      frontLeft.getPosition(), frontRight.getPosition(),
      backLeft.getPosition(), backRight.getPosition()}
      );
    SmartDashboard.putNumber("Robot Heading", getHeading());
}
*/
public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}
//They are all scrambled up, but this is the correct order of the states list
 public void setModuleState(SwerveModuleState[] requestedState) {
    frontLeft.setState(requestedState[1]);
   frontRight.setState(requestedState[0]);
    backLeft.setState(requestedState[3]);
    backRight.setState(requestedState[2]);
 }
}
