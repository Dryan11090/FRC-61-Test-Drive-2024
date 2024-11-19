package frc.robot.Commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;



public class DriverJoysticks extends Command{
   public Joystick turnJoystick = new Joystick(0);  
   public Joystick driveJoystick = new Joystick(1);

  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.PhysicalMaxAcclerationUnitsPerSecond);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.PhysicalMaxAcclerationUnitsPerSecond);
  SlewRateLimiter turningLimiter = new SlewRateLimiter(Constants.PhysicalMaxAcclerationUnitsPerSecond);

  //initialize output variables (unit: m/s)
  double xSpeed = 0;
  double ySpeed = 0;
  double turningSpeed = 0; 

  SwerveSubsystem DriveBase;

    @Override
     public void initialize() {
    
    }

    @Override
    public void execute() {
        DriveBase.setModuleState(this.getFieldRelativeChasisSpeed(DriveBase.getRotation2d()));
        
        
    }

    public  SwerveModuleState[] getFieldRelativeChasisSpeed(Rotation2d RobotAngle) {
        return Constants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(getXSpeed(), -getYSpeed(), getTurningSpeed(), RobotAngle)); 
    }
    //Notes about ^getFieldRelativeChasisSpeed^
        //Take the outputs of the Joysticks and convert them into a formate accepted by SwerveSubsystem
        //FieldRelative is like driving the robot in a "3th person mode"




    //Take the outputs of the Joysticks and convert them into a formate accepted by SwerveSubsystem
    // Relative is relative to the robot, like a 1st person mode
    public  SwerveModuleState[] getRobotRelativeChasisState() {
    return  Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(getXSpeed(), getYSpeed(), getTurningSpeed())); 
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
