package frc.robot;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class MathSwerveModule {
    //Motors
    private CANSparkMax driveMotor;
        private RelativeEncoder driveEncoder;
    private CANSparkMax angleMotor;
        private RelativeEncoder angleEncoder;

    private CANCoder angleAbsoluteEncoder;
    private CANCoderConfiguration configs;

    private double currentAngle;

    public MathSwerveModule(
        int driveMotorID, 
        int angleMotorID, 
        int angleAbsoluteEncoderID, 
        double angleOffset){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
            driveEncoder = driveMotor.getEncoder();
        angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
            angleEncoder = angleMotor.getEncoder();
            angleEncoder.setPositionConversionFactor(Constants.ANGLE_GEAR_RATIO);
        
        angleAbsoluteEncoder = new CANCoder(angleAbsoluteEncoderID);
        configs = new CANCoderConfiguration();
        configs.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        configs.sensorDirection = Constants.CANCODER_INVERTED;
        configs.initializationStrategy = SensorInitializationStrategy.BootToZero;
        configs.sensorTimeBase = SensorTimeBase.PerSecond;
        configs.magnetOffsetDegrees = angleOffset;
    }


    public void setDriveSpeed(double x, double y){
        double speed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        driveMotor.set(speed);
    }
    public double getDriveSpeed(){
        return driveMotor.get();
    }
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }

    public void setAngleSpeed(double x, double y){
        double wantedAngleDegrees = Math.pow(Math.atan(x/y), 2) * 180/Math.PI;
        double speed = Math.abs(Math.abs(wantedAngleDegrees) - Math.abs(currentAngle)) / 180;
        if(speed <= 0.01){
            speed = 0;
        } else if(wantedAngleDegrees > 0 && currentAngle > 0){
            if(wantedAngleDegrees - currentAngle > 0){
                speed *= 1;
            }else{
                speed *= -1;
            }
        } else if(wantedAngleDegrees < 0 && currentAngle < 0){
            if(Math.abs(wantedAngleDegrees) - Math.abs(currentAngle) > 0){
                speed *= 1;
            }else{
                speed *= -1;
            }
        } else if(wantedAngleDegrees > 0 && currentAngle < 0){
            if(Math.abs(wantedAngleDegrees) - Math.abs(currentAngle) > 0){
                if(Math.abs(wantedAngleDegrees) < 90){
                    speed *= -1;
                } else{
                    if(Math.abs(wantedAngleDegrees) - Math.abs(currentAngle) < 90){
                        speed *= 1;
                    }else{
                        speed *= -1;
                    }
                }
            }
        } else if(wantedAngleDegrees < 0 && currentAngle > 0){
            if(Math.abs(wantedAngleDegrees) - Math.abs(currentAngle) > 0){
                if(Math.abs(wantedAngleDegrees) < 90){
                    speed *= 1;
                } else{
                    if(Math.abs(wantedAngleDegrees) - Math.abs(currentAngle) < 90){
                        speed *= -1;
                    }else{
                        speed *= 1;
                    }
                }
            }
        }
        angleMotor.set(speed);
    }
    public void setCurrentAngle(double angle){
        currentAngle = angle;
    }
    public double getCurrentAngle(){
        return angleAbsoluteEncoder.getAbsolutePosition();
    }
}
