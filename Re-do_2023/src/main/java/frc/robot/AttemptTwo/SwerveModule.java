// package frc.robot.AttemptTwo;

// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import frc.robot.Constants;
// import frc.robot.AttemptTwo.lib.CTREModuleState;
// import frc.robot.AttemptTwo.lib.Conversions;
// import frc.robot.AttemptTwo.lib.SwerveModuleConstants;

// public class SwerveModule {
//     public int moduleNumber;
//     private double angleOffset;
//     private CANSparkMax driveMotor;
//     private CANSparkMax angleMotor;
//     private CANCoder angleEncoder;
//     private double lastAngle;
//     private PIDController angleController;
//     private RelativeEncoder angleRelativeEncoder;
//     private RelativeEncoder driveRelativeEncoder;
//     private final CTREConfigs ctreConfigs = new CTREConfigs();
    
//     public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
//         this.moduleNumber = moduleNumber;
//         driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
//         angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
//         angleRelativeEncoder = angleMotor.getEncoder();
//         driveRelativeEncoder = driveMotor.getEncoder();
        
//         angleOffset = moduleConstants.angleOffset;

//         angleController = new PIDController(Constants.ANGLE_KP, Constants.ANGLE_KI, Constants.ANGLE_KD);

//         angleEncoder = new CANCoder(moduleConstants.cancoderID);
//         configAngleEncoder();
//         configAngleMotor();
//         configDriveMotor();

//         lastAngle = getState().angle.getDegrees();
//         }

//         public double angleSpeed(double wantedAngle, double currentAngle){
//             double speed = Math.abs(wantedAngle / currentAngle);
//             if(wantedAngle < 0){
//                 if (currentAngle < 180){
//                     if(wantedAngle > currentAngle){
                        
//                     }else{

//                     }
//                 }else{
//                     if(wantedAngle > currentAngle){
                        
//                     }else{
                        
//                     }
//                 }
//             }else{
//                 if(currentAngle < 180){
//                     if(wantedAngle > currentAngle){
                        
//                     }else{
                        
//                     }
//                 }else{
//                     if(wantedAngle > currentAngle){
                        
//                     }else{
                        
//                     }
//                 }
//             }
//             return speed;
//         }

//     public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
//         desiredState = CTREModuleState.optimize(desiredState, getState().angle);

//         if(isOpenLoop){
//             driveMotor.set(desiredState.speedMetersPerSecond / Constants.DRIVE_MAX_SPEED);
//         }else{
//             driveMotor.set(Conversions.MPSToNeo(desiredState.speedMetersPerSecond, Constants.DRIVE_WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO));
//         }
//         double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.DRIVE_MAX_SPEED * 0.02)) ? lastAngle : desiredState.angle.getDegrees();
//         angleMotor.set(angleController.calculate(lastAngle, Conversions.degreesToNeo(angle, Constants.ANGLE_GEAR_RATIO)));
//         lastAngle = angle;
//     }

//     public void resetToAbsolute(){
//         double absolutePosition = Conversions.degreesToNeo(getCancoder().getDegrees() - angleOffset, Constants.ANGLE_GEAR_RATIO);
//         angleEncoder.setPosition(absolutePosition);
//     }

//     private void configAngleEncoder(){
//         angleEncoder.configFactoryDefault();
//         angleEncoder.configAllSettings(CTREConfigs.canCoderConfig);
//     }

//     private void configAngleMotor(){
//         angleController.setP(Constants.ANGLE_KP);
//         angleController.setI(Constants.ANGLE_KI);
//         angleController.setD(Constants.ANGLE_KD);
//         angleController.setTolerance(0.1);
//         angleController.enableContinuousInput(-Math.PI, Math.PI);
//         angleMotor.setInverted(Constants.ANGLE_MOTOR_INVERTED);
//         resetToAbsolute();
//     }
//     private void configDriveMotor(){
//         driveMotor.setInverted(Constants.DRIVE_MOTOR_INVERTED);
//     }

//     public Rotation2d getCancoder(){
//         return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
//     }

//     public SwerveModuleState getState(){
//         double velocity = Conversions.neoToMPS(driveMotor.get(), Constants.DRIVE_WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
//         Rotation2d angle = Rotation2d.fromDegrees(Conversions.neoToDegrees(angleRelativeEncoder.getPosition(), Constants.ANGLE_GEAR_RATIO));
//         return new SwerveModuleState(velocity, angle);
//     }

//     private Rotation2d getAngle(){
//         return Rotation2d.fromDegrees(Conversions.neoToDegrees(angleRelativeEncoder.getPosition(), Constants.ANGLE_GEAR_RATIO));
//     }

//     public SwerveModulePosition getPosition(){
//         return new SwerveModulePosition(
//             Conversions.neoToMeters(driveRelativeEncoder.getPosition(), Constants.DRIVE_WHEEL_CIRCUMFERENCE), getAngle());
//     }
// }
