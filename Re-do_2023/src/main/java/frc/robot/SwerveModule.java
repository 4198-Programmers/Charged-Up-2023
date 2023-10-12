package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**This makes each swerve module */
public class SwerveModule {
    //initializing motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;
    //initializing the relative encoders for each motor
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;
    /*
    This class handles the feedback loop calculation for the user, as
    well as offering methods for returning the error, setting tolerances,
    and checking if the control loop has reached its setpoint within the specific
    tolerances.

    Starting with kP is a good idea, mainly because it is the most intuitive, 
    it will scale your output proportionally to your error. Start with a very small value. 
    In general, kP shouldn’t be greater than 1/(max error) because this would attempt to drive 
    your output to max power. This is relative of course to how you normalize your inputs/outputs, 
    but unless you are doing something special, make sure you follow this rule.

    My preferred method of tuning is to keep a small kP, and slowly dial it up until I am close 
    to my goal but not quite there. Then I slowly increment kI. The reason I do this is because 
    it can be shown mathematically that any value of kI will drive your steady state error to 0. 
    This does not mean that you should choose kI poorly, as there is a variety of stability issues 
    you can cause with a big kI.

    I don’t use kD very often, but this is more preference. You can basically do the “opposite” of my approach 
    - tune kP until a little overshoot and then up kD to decrease it.
    */
    /**
     * kp - the proportional coefficient; have the effect of reducing the rise time and will reduce ,but never eliminate.<p>
     * ki - the integral coefficient; the effect of eliminating the steady-state error, but it may make the transient response worse<p>
     * kd - the derivative coefficient; the effect of increasing the stability of the system, reducing the overshoot, and improving the transient response
     */
    private final PIDController anglePIDController;
    /*
    We need the analog input because when the robot gets turned off, it 
    looses the relative encoder values.
     */
    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    /*
    We need to get offsets because the readings will be "wrong"
    from what is supposed to be the front.
    */

    public SwerveModule(int driveMotorID, int angleMotorID, boolean driveMotorReversed, boolean anglemotorReversed,
        int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
            driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
            angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            absoluteEncoder = new CANCoder(absoluteEncoderID);
            //everything in degrees
            absoluteEncoder.configMagnetOffset(absoluteEncoderOffset);
            absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            //Will curently not be inverted
            driveMotor.setInverted(driveMotorReversed);
            angleMotor.setInverted(anglemotorReversed);
            
            driveEncoder = driveMotor.getEncoder();
            angleEncoder = angleMotor.getEncoder();

            // driveEncoder.setPositionConversionFactor(Constants.DriveEcoderRotationToMeter);
            // driveEncoder.setVelocityConversionFactor(Constants.DriveEncoderRPMToMeterPerSec);

            // angleEncoder.setPositionConversionFactor(Constants.AngleEncoderRotationToRadian);
            // angleEncoder.setVelocityConversionFactor(Constants.AngleEncoderRPMToRadPerSec);
            
            anglePIDController = new PIDController(Constants.kp, Constants.ki, Constants.kd);
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
            gets the angle of the robot modules
            */
            double angle = absoluteEncoder.getAbsolutePosition();
            //Multiply it by 2PI to convert to radians.
            angle*=2.0*Math.PI;
            //Subtract offset to get the actual wheel angles.
            return angle * (absoluteEncoderReversed ? -1 : 1);
        }

        public void resetEncoders(){
            driveEncoder.setPosition(0);
            angleEncoder.setPosition(getAbsoluteEncoderRad());
        }

        public SwerveModuleState getState(){
            return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAnglePosition()));
        }
        public void setDesiredState(SwerveModuleState state){
            if (Math.abs(state.speedMetersPerSecond) < 0.1){
                stop();
                return;
            }
            state = SwerveModuleState.optimize(state, getState().angle);
            driveMotor.set(state.speedMetersPerSecond / Constants.MAX_SPEED_METERS_PER_SECOND);
            angleMotor.set(anglePIDController.calculate(getAnglePosition(), state.angle.getRadians()));
            SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "]", state.toString());
        }

        public SwerveModulePosition getPosition(){
            return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
        }

    public void stop(){
        driveMotor.set(0);
        angleMotor.set(0);
    }
}