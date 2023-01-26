package frc.robot.wpiVcontainer;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Maths;

public class FieldSwerveModule {
    private CANSparkMax driveMotor; // motor that drives the wheel
    private CANSparkMax spinMotor;// motor that spins the wheel around to face different ways
    private CANCoder spinEncoder; // cancoder, just named enco
    private RelativeEncoder driveEncoder; // encoder for drvie motor
    private double spinOffset; // distance from our zero to the magnetic 0 in the cancoders
    private double spinPos; // output of the cancoder, from 0-360
    private Rotation2d rotation; // turns cancoder value to valid rotation value for later use
    private double encoderDistance; // distance in meters that the drive motor has moved
    double maxSpinAcceleration = 0.002; // in meters per second
    double maxSpinVelocity = 0.4667056958; // in meters per second
    double maxDriveVelocity = 0.4667056958; // in meters per second
    double maxDriveAcceleration = 0.002; // in meters per second
    double spinkP = 0.000025;
    double spinkI = 0.000001;
    double spinkD = 0.00001;
    double drivekP = 0.0001;
    double drivekI = 0.0000001;
    double drivekD = 0;
    // private ProfiledPIDController spinPID = new ProfiledPIDController(spinkP,
    // spinkI, spinkD,
    // new TrapezoidProfile.Constraints(maxSpinVelocity, maxSpinAcceleration));
    private PIDController spinPID = new PIDController(spinkP, spinkI, spinkD);
    private PIDController drivePID = new PIDController(drivekP, drivekI, drivekD);

    public FieldSwerveModule(CANSparkMax driveMotorArg, int spinID, int CANCoderID, double CANCoderZeroOffset) {
        driveMotor = driveMotorArg;
        spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
        spinEncoder = new CANCoder(CANCoderID);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(8.14);
        // spinEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        spinEncoder.configMagnetOffset(spinOffset);
    }

    public void PIDStart() {
        drivePID.reset();
        // spinPID.reset(null);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // double wantedAngle = desiredState.angle.getDegrees();
        Rotation2d rotation = new Rotation2d(spinPos);
        spinPID.enableContinuousInput(-180, 180);
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, rotation);
        double wantedAngle = state.angle.getDegrees();
        // byte spinOptimize;
        // if (Math.abs(spinPos - wantedAngle)>90){
        // spinOptimize = -1;
        // } else{
        // spinOptimize = 1;
        // }
        // SwerveModuleState state = SwerveModuleState.optimize(desiredState, rotation);
        spinPos = spinEncoder.getAbsolutePosition();
        double driveOut = drivePID.calculate(driveMotor.get(),
                desiredState.speedMetersPerSecond);
        driveMotor.set(driveOut / maxDriveVelocity);
        double turnOut = spinPID.calculate(spinPos, wantedAngle);
        spinMotor.set((turnOut / maxSpinVelocity));

        // if (spinPos < wantedAngle) {// desiredState.angle.getDegrees()
        // spinMotor.set(-0.02 * spinOptimize);
        // } else if (spinPos > wantedAngle) {
        // spinMotor.set(0.02 * spinOptimize);
        // } else {
        // spinMotor.set(0);
        // }

        // driveMotor.set((state.speedMetersPerSecond/maxDriveVelocity)/8);

    }

    public SwerveModulePosition getPosition() {
        if (spinPos < 0) {
            spinPos = 360 + spinEncoder.getAbsolutePosition() - spinOffset; // makes sure that the number stays between
                                                                            // 0-360 instead of going negative
        } else if (spinPos >= 0) {
            spinPos = spinEncoder.getAbsolutePosition() - spinOffset;
        } else {
            System.out.println("xdr");
        }
        rotation = Rotation2d.fromDegrees(spinPos);
        encoderDistance = Maths.positionConversion(driveEncoder.getPosition());
        return new SwerveModulePosition(encoderDistance, rotation);

    }

}
