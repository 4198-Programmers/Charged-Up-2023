package frc.robot;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.AutoConstants;
import frc.robot.Subsystems.Swerve;

public class SwerveModuleTest{
    private final CANSparkMax driveMotor;
    private final CANSparkMax angleMotor;
    private final CANCoder angleEncoder;
    private final PIDController anglePIDController;

    /**The targe wheel angle in rotations [-0.5, 0.5] */
    private double desiredAngle;
    /**The target wheel drive speed in rotations per second */
    private double desiredDriveSpeed;

    ShuffleboardTab debugInfo;

    public SwerveModuleTest(
        String moduleName,
        int driveMotorID,
        int angleMotorID,
        int angleEncoderID,
        Rotation2d angleEncoderOffset){
            driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
            angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
            angleEncoder = new CANCoder(angleEncoderID);

            anglePIDController = new PIDController(AutoConstants.kPXController, 0, 0);
            anglePIDController.enableContinuousInput(-Math.PI, Math.PI);

            CANCoderConfiguration angleEncoderConfiguration = Swerve.ctreConfigs.coderConfiguration;
            angleEncoderConfiguration.magnetOffsetDegrees = -angleEncoderOffset.getRotations();
            angleEncoder.getAllConfigs(angleEncoderConfiguration);
        }

        static class GyroView implements Sendable{
            DoubleSupplier value;
            GyroView(DoubleSupplier value){
                this.value = value;
            }
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Gyro");
                builder.addDoubleProperty("Value", value, null);
            }
        }

        public SwerveModuleTest(
            String moduleName,
            ShuffleboardLayout sbLayout,
            boolean debug,
            int driveMotorChannel,
            int steerMotorChannel,
            int steerEncoderChannel,
            Rotation2d steerEncoderOffset) {
          this(moduleName, driveMotorChannel, steerMotorChannel, steerEncoderChannel, steerEncoderOffset);
          
          ShuffleboardLayout MotorInfo = sbLayout.getLayout("General Information", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of Columns", 1, "Number of rows",  3, "Label Position", "LEFT"));

          ShuffleboardLayout ModuleInfo = sbLayout.getLayout("Module Information", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of Columns", 1, "Number of rows",  4, "Label Position", "Right"));
            MotorInfo.add(moduleName + " Angle", new GyroView(() -> MathUtil.inputModulus(getRotation().getDegrees(), -180.0, 180.0)))
            .withWidget(BuiltInWidgets.kGyro)
            .withProperties(Map.of("Major tick spacing", 180, "Counter Clockwise", true));

            MotorInfo.addNumber(moduleName + " Drive Voltage", () -> driveMotor.getOutputCurrent());
            MotorInfo.addNumber(moduleName + " Angle", () -> getRotation().getDegrees());

            ModuleInfo.addNumber(moduleName + " Speed", () -> getSpeedMetersPerSecond());
            ModuleInfo.addNumber(moduleName + " Angle", () -> getRotation().getDegrees());
            ModuleInfo.addNumber(moduleName + " Speed Target", () -> getTargetSpeed());
            ModuleInfo.addNumber(moduleName + " Angle Target", () -> getTargetAngle());

            if(debug){
                debugInfo = Shuffleboard.getTab("Debug");
                debugInfo.addNumber("Drive Target", () -> driveMotor.getClosedLoopRampRate());
                debugInfo.addNumber("Drive Value", () -> driveMotor.getEncoder().getVelocity());
                
                debugInfo.addNumber("Angle Target", () -> angleMotor.getClosedLoopRampRate());
                debugInfo.addNumber("Angle Value", () -> getRotation().getRotations());
            }
          }

          public SwerveModuleState getState(){
            return new SwerveModuleState(getSpeedMetersPerSecond(), getRotation());
          }

          public SwerveModulePosition getPosition(){
            return new SwerveModulePosition(getDriveDistance(), getRotation());
          }

          public Rotation2d getRotation(){
            return Rotation2d.fromRotations(MathUtil.inputModulus(angleMotor.getEncoder().getPosition(), -0.5, 0.5));
          }

          public double getTargetAngle(){
            return desiredAngle;
          }

          public double getTargetSpeed(){
            return desiredDriveSpeed;
          }

          public double getSpeedMetersPerSecond(){
            return (driveMotor.getEncoder().getVelocity() * Constants.DRIVE_ROTATIONS_TO_METERS);
          }

          public double getDriveDistance(){
            return (driveMotor.getEncoder().getPosition() * Constants.DRIVE_ROTATIONS_TO_METERS);
          }

          public void stop(){
            driveMotor.set(0);
            angleMotor.set(0);
          }

          public void setDesiredState(SwerveModuleState desiredState){
            if(Math.abs(desiredState.speedMetersPerSecond) < 0.001){
                stop();
                return;
              }
            if(desiredState.angle == null){
                DriverStation.reportWarning("Cannot set module angle to null.", true);
            }

            SwerveModuleState state = SwerveModuleState.optimize(desiredState, getRotation());
            desiredAngle = MathUtil.inputModulus(desiredState.angle.getRotations(), -0.5, 0.5);
            desiredDriveSpeed = desiredState.speedMetersPerSecond / Constants.DRIVE_ROTATIONS_TO_METERS;

            driveMotor.set(desiredDriveSpeed);
            angleMotor.set(anglePIDController.calculate(state.angle.getRotations(), desiredAngle));
          }
}
