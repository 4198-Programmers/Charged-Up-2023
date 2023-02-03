package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;


public class GyroSub extends SubsystemBase {
    AHRS gyro = new AHRS(Port.kMXP);
    
    public Rotation2d getRotation2dManual() {
        return new Rotation2d(gyro.getYaw());
    }

    public void displayGyro(){
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
        Shuffleboard.getTab("Gyro Tab").add(gyro);
    }

    public void resetGyro(){
        gyro.reset();
    }



}
