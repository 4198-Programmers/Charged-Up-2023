package frc.robot;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
/**This makes each swerve module */
public class SwerveModule {
    //initializing motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;
    //initializing the relative encoders for each motor
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;

    private final PIDController anglePIDController;
    /*
    We need the analog input because when the robot gets turned off, it 
    looses the relative encoder values.
     */
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    //We need to get offsets because the readings will be "wrong"
    //from what is supposed to be the front.
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean anglemotorReversed,
        int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
            driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
            angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
            absoluteEncoderOffsetRad = absoluteEncoderOffset;
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            absoluteEncoder = new AnalogInput(absoluteEncoderID);

            driveMotor.setInverted(driveMotorReversed);
            angleMotor.setInverted(anglemotorReversed);
            
            driveEncoder = driveMotor.getEncoder();
            angleEncoder = angleMotor.getEncoder();

            driveEncoder.setPositionConversionFactor(Constants.DriveEcoderRotationToMeter);
            driveEncoder.setVelocityConversionFactor(Constants.DriveEncoderRPMToMeterPerSec);

            angleEncoder.setPositionConversionFactor(Constants.AngleEncoderRotationToRadian);
            angleEncoder.setVelocityConversionFactor(Constants.AngleEncdoerRPMToRadPerSec);

            anglePIDController = new PIDController(Constants.Angle, 0, 0);
            anglePIDController.enableContinuousInput(-Math.PI, Math.PI);
        }
        public double getDrivePosition(){
            return driveEncoder.getPosition();
        }

        public double getAnglePosition(){
            return angleEncoder.getPosition();
        }

        public double getDriveVelocity(){
            return driveEncoder.getVelocity();
        }

        public double getAngleVelocity(){
            return angleEncoder.getVelocity();
        }

        public double getAbsoluteEncoderRad(){
            /*
            to get the absolute in total value we'll need to divide
            its voltage reading by the voltage that we're supplying it
            that gives us how many percent of a full rotation it is reading
            */
            double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
            //Multiply it by 2PI to convert to radians.
            angle*=2.0*Math.PI;
            //Subtract offset to get the actual wheel angles.
            angle-=absoluteEncoderOffsetRad;
            return angle * (absoluteEncoderReversed ? -1 : 1);
        }

        public void resetEncoders(){
            driveEncoder.setPosition(0);
            angleEncoder.setPosition(getAbsoluteEncoderRad());
        }
    
}
