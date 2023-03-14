package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.DriveTrain;

public class Balance extends CommandBase {
    private DriveTrain driveTrain;
    float pitch;
    double speed;
    double pitchDouble;
    Timer timer;

    public Balance(DriveTrain driveArg) {
        driveTrain = driveArg;
        addRequirements(driveTrain);
    }
    @Override
    public void initialize() {
        timer = new Timer();
    }

    @Override
    public void execute() {
        pitch = driveTrain.getPitch();
        speed = 0.5;
        pitchDouble = (double) pitch;

        // if (pitchDouble < 70 || pitchDouble > -70 && pitchDouble > 5 && pitchDouble <
        // -5) {
        // speed = ((pitchDouble / 70) * 2);
        // } else if (pitchDouble > 70 || pitchDouble < -70) {
        // speed = 2;
        // } else {
        // speed = 0;
        // }

        // if (pitchDouble > 5) {
        //     speed = 0.5;
        // } else if (pitchDouble > 2 && pitchDouble < 5) {
        //     speed = 0.15;
        // } else if (pitchDouble < -2 && pitchDouble > -5) {
        //     speed = -0.15;
        // } else if (pitchDouble < -5) {
        //     speed = -0.5;
        // } else {
        //     speed = 0;
        // }
        while(timer.get() < 0.5){
           driveTrain.drive(new ChassisSpeeds(speed, 0, 0));
           System.out.println("Balance " + timer.get());
        }
        
        timer.reset();
        while(timer.get() < 0.5){
            driveTrain.drive(new ChassisSpeeds(0, 0, 0));
            System.out.println("Stop " + timer.get());
        }


    }

    @Override
    public boolean isFinished() {
        return pitchDouble >= -0.5 && pitchDouble <= 0.5;
        // return driveTrain.BalanceDrive() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(new ChassisSpeeds(0, 0, 0));
    }

}
