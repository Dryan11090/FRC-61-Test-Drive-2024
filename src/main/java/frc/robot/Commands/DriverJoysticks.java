package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

//TODO Fix Wheel rotating after joystick returns from doing a half spin on the input
//Just add "&& driveJoystick.isConnected()" NOT TESTED
// Removed double negitives on getYSpeed and getThataSpeed
//Added SlewRateLimiters to getXSpeed, getYSpeed, getTurningSpeed 

public class DriverJoysticks {
  //Get the joysticks from DriverStation
   public Joystick driveJoystick = new Joystick(1);
   public Joystick turnJoystick = new Joystick(0);

  //Create acceleration limit
  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.PhysicalMaxAcclerationUnitsPerSecond);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.PhysicalMaxAcclerationUnitsPerSecond);
  SlewRateLimiter turningLimiter = new SlewRateLimiter(Constants.PhysicalMaxAcclerationUnitsPerSecond);

  //initialize output variables (unit: m/s)
  double xSpeed = 0;
  double ySpeed = 0;
  //TODO find out what unit this is 
    double turningSpeed = 0;


    //Take the outputs of the Joysticks and convert them into a formate accepted by SwerveSubsystem
    // Relative is relative to the robot, like a 1st person mode
    public  SwerveModuleState[] getRobotRelativeChasisState() {
    return  Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(getXSpeed(), getYSpeed(), getTurningSpeed())); 
    }

    //Take the outputs of the Joysticks and convert them into a formate accepted by SwerveSubsystem
    //FieldRelative is like driving the robot in a "3th person mode"
    public  SwerveModuleState[] getFieldRelativeChasisSpeed(Rotation2d RobotAngle) {
        return Constants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(getXSpeed(), -getYSpeed(), getTurningSpeed(), RobotAngle)); 
    }

    public double getXSpeed() {
//if not in deadzone and controller is connected
         if(Math.abs(driveJoystick.getY()) > 0.17 && driveJoystick.isConnected()) {
         return xLimiter.calculate(driveJoystick.getY()*Constants.topSpeedX);    
        }
        return 0;
    }

    public double getYSpeed() {
        if(Math.abs(driveJoystick.getX()) > 0.17 && driveJoystick.isConnected()) {
            return yLimiter.calculate(driveJoystick.getX()*Constants.topSpeedY);    
        }
        return 0;
    }

    public double getTurningSpeed() {
        if(Math.abs(turnJoystick.getX()) > 0.17 && turnJoystick.isConnected()) {
         return turningLimiter.calculate(turnJoystick.getX()*Constants.topSpeedTurning);    
        }
        return 0;
    }
}
