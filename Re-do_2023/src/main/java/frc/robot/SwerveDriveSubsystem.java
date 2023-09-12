package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase{
    //Motors
    private static CANSparkMax LEFT_FRONT_DRIVE_SPEED_MOTOR;
    private static CANSparkMax LEFT_BACK_DRIVE_SPEED_MOTOR;
    private static CANSparkMax RIGHT_FRONT_DRIVE_SPEED_MOTOR;
    private static CANSparkMax RIGHT_BACK_DRIVE_SPEED_MOTOR;

    private static CANSparkMax LEFT_FRONT_DIRECTION_MOTOR;
    private static CANSparkMax LEFT_BACK_DIRECTION_MOTOR;
    private static CANSparkMax RIGHT_FRONT_DIRECTION_MOTOR;
    private static CANSparkMax RIGHT_BACK_DIRECTION_MOTOR;

    //Encoders
    public static Encoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
    public static Encoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
    public static Encoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
    public static Encoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;

    public static Encoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
    public static Encoder LEFT_BACK_DRIVE_DIRECTION_ENCODER;
    public static Encoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
    public static Encoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;

    //Gyro
    public static AHRS DRIVE_GYRO;

    public void Drive(){

        LEFT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.FRONT_LEFT_DRIVE, MotorType.kBrushless);
        LEFT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.BACK_LEFT_DRIVE, MotorType.kBrushless);
        RIGHT_FRONT_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
        RIGHT_BACK_DRIVE_SPEED_MOTOR = new CANSparkMax(Constants.BACK_RIGHT_DRIVE, MotorType.kBrushless);

        LEFT_FRONT_DIRECTION_MOTOR = new CANSparkMax(Constants.FRONT_LEFT_STEER, MotorType.kBrushless);
        LEFT_BACK_DIRECTION_MOTOR = new CANSparkMax(Constants.BACK_LEFT_STEER, MotorType.kBrushless);
        RIGHT_FRONT_DIRECTION_MOTOR = new CANSparkMax(Constants.FRONT_RIGHT_STEER, MotorType.kBrushless);
        RIGHT_BACK_DIRECTION_MOTOR = new CANSparkMax(Constants.BACK_RIGHT_STEER, MotorType.kBrushless);

        //Encoders
        

    }


}
