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
import frc.robot.Constants;
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
    double spinkP = 1;
    double spinkI = 0.01;
    double spinkD = 0.001;
    double drivekP = 1;
    double drivekI = 0.01;
    double drivekD = 0.001;
    // private ProfiledPIDController spinPID = new ProfiledPIDController(spinkP,
    // spinkI, spinkD,
    // new TrapezoidProfile.Constraints(maxSpinVelocity, maxSpinAcceleration));
    private PIDController spinPID = new PIDController(spinkP, spinkI, spinkD);
    private PIDController drivePID = new PIDController(drivekP, drivekI, drivekD);
private int spinID;
    public FieldSwerveModule(CANSparkMax driveMotorArg, int spinID, int CANCoderID, double CANCoderZeroOffset) {
        this.spinID = spinID;
        driveMotor = driveMotorArg;
        spinOffset = CANCoderZeroOffset;
        spinMotor = new CANSparkMax(spinID, MotorType.kBrushless);
        spinEncoder = new CANCoder(CANCoderID);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(8.14);
        spinPID.enableContinuousInput(-180, 180);
        spinEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        spinEncoder.configMagnetOffset(spinOffset);
    }

    public void PIDStart() {
        drivePID.reset();
        // spinPID.reset(null);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // double wantedAngle = desiredState.angle.getDegrees();
        spinPos = spinEncoder.getAbsolutePosition();
        Rotation2d currentRotation = Rotation2d.fromDegrees(spinPos);
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
        Rotation2d optimizedAngle = state.angle;
        double turnSpeed = optimizedAngle.minus(currentRotation).getDegrees() / 90.0;
        double wantedAngle = state.angle.getDegrees();
        // byte spinOptimize;
        // if (Math.abs(spinPos - wantedAngle)>90){
        // spinOptimize = -1;
        // } else{
        // spinOptimize = 1;
        // }
        // SwerveModuleState state = SwerveModuleState.optimize(desiredState, rotation);
        double driveOut = drivePID.calculate(driveMotor.get(),
                desiredState.speedMetersPerSecond);
        // driveMotor.set(driveOut / maxDriveVelocity);

        // double currentDifferenceInAngle = wantedAngle - spinEncoder.getAbsolutePosition();
        // double differenceOfDifferenceInAngle = currentDifferenceInAngle - 
        // (Constants.WANTED_DIFFERENCE_IN_ANGLE * (currentDifferenceInAngle / Math.abs(currentDifferenceInAngle)));
        double turnOut = spinPID.calculate(spinMotor.get(), turnSpeed);
        spinMotor.set(turnOut * 0.5);
        System.out.println("Module: " + spinID + " turn:" + turnOut + " Turn Speed: " + turnSpeed +  " drive: " + driveOut + " Wanted: " + wantedAngle + " Current: " + spinPos);

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
