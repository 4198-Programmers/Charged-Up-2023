// package frc.robot;

// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.DriverStation;
// import frc.lib.config.CTREConfigs;

// public class SwerveModuleMythic {
//     private final CANSparkMax driveMotor;
//     private final CANSparkMax angleMotor;

//     private final RelativeEncoder driveEncoder;
//     private final RelativeEncoder angleRelativeEncoder;
//     private final CANCoder angleEncoder;
//     private final Rotation2d angleEncoderOffset;
//     private final CTREConfigs configs = new CTREConfigs();
    
//     private double desiredAngle;
//     private double desiredDriveSpeed;

//     public SwerveModuleMythic(String moduleName,
//     int driveMotorID,
//     int angleMotorID,
//     int angleEncoderID,
//     Rotation2d angleOffset){
//         driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
//         angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);

//         driveEncoder = driveMotor.getEncoder();
//         angleRelativeEncoder = angleMotor.getEncoder();
//         angleEncoder = new CANCoder(angleEncoderID);
//         angleEncoderOffset = angleOffset;

//         CANCoderConfiguration angleEncoderConfig = configs.swerveCanCoderConfig;
//         angleEncoderConfig.magnetOffsetDegrees = -angleEncoderOffset.getRotations();
//     }

//     public SwerveModuleState getState(){
//         return new SwerveModuleState(getSpeed(), getRotation());
//     }

//     public SwerveModulePosition getPosition(){
//         return new SwerveModulePosition(getDriveDistance(), getRotation());
//     }

//     public Rotation2d getRotation(){
//         return Rotation2d.fromRotations(MathUtil.inputModulus(angleRelativeEncoder.getPosition(), -0.5, 0.5));
//     }

//     public double getTargetAngle(){
//         return desiredAngle;
//     }

//     public double getTargetSpeed(){
//         return desiredDriveSpeed;
//     }

//     public double getSpeed(){
//         return (driveEncoder.getVelocity() * Constants.DRIVE_ROTATIONS_TO_METERS);
//     }

//     public double getDriveDistance(){
//         return (driveEncoder.getPosition() * Constants.DRIVE_ROTATIONS_TO_METERS);
//     }

//     public void setDesiredState(SwerveModuleState desiredState){
//         if(desiredState.angle == null){
//             DriverStation.reportError("Cannot Set Module Angle to Null", true);
//         }
//         SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation());
//         desiredAngle = MathUtil.inputModulus(state.angle.getRotations(), -0.5, 0.5);
//         desiredDriveSpeed = state.speedMetersPerSecond / Constants.DRIVE_ROTATIONS_TO_METERS;

//         driveMotor.setControl
//     }
//}
