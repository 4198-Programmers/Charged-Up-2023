package frc.robot.wpiDrive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
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
    // double maxSpinVelocity = 0.4667056958; // in meters per second
    double maxSpinVelocity = 3.5;
    // double maxDriveVelocity = 0.4667056958; // in meters per second
    double maxDriveVelocity = 3.5;
    double maxDriveAcceleration = 0.002; // in meters per second
    // double spinkP = 0.0025;//0.005
    // double spinkI = 0;
    // double spinkD = 0.000008;
    double drivekP = 0.0001;
    double drivekI = 0.0000001;
    double drivekD = 0;
    // private ProfiledPIDController spinPID = new ProfiledPIDController(spinkP,
    // spinkI, spinkD,
    // new TrapezoidProfile.Constraints(maxSpinVelocity, maxSpinAcceleration));
    private PIDController spinPID = new PIDController(1.0, 0.0, 0.1);
    private PIDController drivePID = new PIDController(drivekP, drivekI, drivekD);

    public FieldSwerveModule(CANSparkMax driveMotorArg, int spinID, int CANCoderID, double CANCoderZeroOffset) {
        driveMotor = driveMotorArg;
        spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
        spinEncoder = new CANCoder(CANCoderID);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(8.14);
        spinEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        spinEncoder.configMagnetOffset(spinOffset);
        spinPID.setPID(1.0, 0.0, 0.1);
        drivePID.setPID(drivekP, drivekI, drivekD);
        spinPID.enableContinuousInput(-180, 180);
    }

    public void PIDStart() {
        drivePID.reset();
        // spinPID.reset(null);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(spinEncoder.getAbsolutePosition()));
        double driveOut = drivePID.calculate(driveMotor.get(),
                (state.speedMetersPerSecond / maxDriveVelocity));
        // driveMotor.set(driveOut / maxDriveVelocity);
        double turnOut = spinPID.calculate(spinEncoder.getAbsolutePosition(), 0);// state.angle.getDegrees()
        spinMotor.set((turnOut / maxSpinVelocity));
        System.out.println(turnOut / maxSpinVelocity);
    }

    public SwerveModulePosition getPosition() {
        // if (spinPos < 0) {
        // spinPos = 360 + spinEncoder.getAbsolutePosition() - spinOffset; // makes sure
        // that the number stays between
        // // 0-360 instead of going negative
        // } else if (spinPos >= 0) {
        // spinPos = spinEncoder.getAbsolutePosition() - spinOffset;
        // } else {
        // System.out.println("xdr");
        // }
        rotation = Rotation2d.fromDegrees(spinPos);
        encoderDistance = Maths.positionConversion(driveEncoder.getPosition());
        return new SwerveModulePosition(encoderDistance, rotation);

    }

}
