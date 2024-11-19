package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;


public class SwerveModule {
   public TalonFX driveMotor, turningMotor;
   public CANcoder absEncoder;
   private boolean driveMotorReversed, turningMotorReversed;

   private boolean absEncoderReversed;
   private double absEncoderOffsetRad;

   private PIDController turningPID;

   private final NeutralOut m_brake = new NeutralOut(); 

    TalonFXConfiguration Configs;

    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  


   
   public SwerveModule(int driveMotorID,int turningMoterID, Boolean inputdriveMotorReversed, Boolean inputturningMotorReversed,
    int absEncoderID, double absEncoderOFFSET, boolean absEncoderREVERSED) {
       this.absEncoder = new CANcoder(absEncoderID);
        this.absEncoderOffsetRad = absEncoderOFFSET;
        this.absEncoderReversed = absEncoderREVERSED;

        this.driveMotorReversed = inputdriveMotorReversed;
        this.turningMotorReversed = inputturningMotorReversed;

        driveMotor = new TalonFX(driveMotorID);
        turningMotor = new TalonFX(turningMoterID);

        turningPID = new PIDController(Constants.TurningProportionalGain, Constants.TurningReset, 0);
        turningPID.enableContinuousInput(Math.PI, -Math.PI);

        resetEncoders();

        this.Configs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        Configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        Configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        Configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        Configs.Slot0.kI = 0; // No output for integrated error
        Configs.Slot0.kD = 0; // No output for error derivative
        // Peak output of 8 volts
        Configs.Voltage.withPeakForwardVoltage(Volts.of(8).magnitude())
          .withPeakReverseVoltage(Volts.of(-8).magnitude());

              StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
                status = driveMotor.getConfigurator().apply(Configs);
                if (status.isOK()) break;
            }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }
    //untested
    public double getDrivePosition() {
        StatusSignal<Double> drivingPosition = driveMotor.getPosition();
        return drivingPosition.getValueAsDouble();
    } 
    //untested
    public double getTurningPosition() {
        StatusSignal<Double> turningPosition = turningMotor.getPosition();
        return turningPosition.getValueAsDouble();
    }
    //untested
    public double getDriveVelocity() {
        StatusSignal<Double> drivingVerlocity = driveMotor.getVelocity();
        return drivingVerlocity.getValueAsDouble();
    }
    //untested
    public double getTurningVelocity() {
        StatusSignal<Double> turningVerlocity = turningMotor.getVelocity();
        return turningVerlocity.getValueAsDouble();
    }

    public double getAbsEncoderValue() {
        StatusSignal<Double> absEncoderReading = this.absEncoder.getAbsolutePosition(); //Get Reading from encoder
        double angle = absEncoderReading.getValueAsDouble(); //Put the value in a local variable
        angle *= 2.0 * Math.PI; // From rotations to Radians
        angle-= absEncoderOffsetRad; //If needed there is an option to add an offset. USE the CTR Phoenix Tuner you can make the 0 value any pos you want
        return angle * (absEncoderReversed ? -1.0 : 1.0); //This is a condensed if statment || Reversed = true then *-1 || Reversed = false then *1
        //^This check is inplace incase build team places the encoder OR THE TURN MOTOR upside down^
    }
//this has no effect on the absEncoder reading only rests the interal encoders of the motors
    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningMotor.setPosition(getAbsEncoderValue());
    }

//this is a test methoid that test what the robot calls Foward
    public void goToZero() {
    driveMotor.set(0.1);
    turningMotor.set(turningPID.calculate(getAbsEncoderValue(), 0));
}

//untested || likely to fail due to the lack of proper vector addtion
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition().getValue(), new Rotation2d(getAbsEncoderValue()));
      }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setState(SwerveModuleState state) {
            if (Math.abs(state.speedMetersPerSecond) < 0.05) {
                stop();
                return;
            }  
       
        state = SwerveModuleState.optimize(state, new Rotation2d(getAbsEncoderValue()));
        // driveMotor.set(-state.speedMetersPerSecond* 2/3 * (driveMotorReversed ? -1 : 1));
        driveMotor.setControl(m_velocityVoltage.withVelocity(-state.speedMetersPerSecond* 80 * (driveMotorReversed ? -1 : 1)));
        double turnSpeed = (turningMotorReversed ? -1 : 1) * turningPID.calculate(getAbsEncoderValue(), state.angle.getRadians());
        turningMotor.set(turnSpeed);
    }   

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    public void halt() {
        driveMotor.setControl(m_brake);
        turningMotor.setControl(m_brake);
    }
    public void close() {
        driveMotor.close();
        turningMotor.close();
        absEncoder.close();
        
    }
}
