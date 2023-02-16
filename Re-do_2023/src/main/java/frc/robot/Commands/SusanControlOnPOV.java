package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.LazySusanSub;

public class SusanControlOnPOV extends CommandBase {
    private final LazySusanSub lazySusan;
    private final DoubleSupplier speedSupplier;
    private final double speedScalar;
    private final int degrees;

    public SusanControlOnPOV(LazySusanSub susanArg, DoubleSupplier supplier, int percentSpeed/*maybe try passing it a double (no need for  "/ 100" and see if it fixes things*/, int degrees) {
        lazySusan = susanArg;
        speedSupplier = supplier;
        speedScalar = percentSpeed / 100;
        this.degrees = degrees;
        addRequirements(lazySusan);
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        if(degrees == -1){
            speed = 0;
        }
        else if(lazySusan.getRotation() < degrees - Constants.ANGLE_OFFSET){
            speed = speedSupplier.getAsDouble();
        }
        else if(lazySusan.getRotation() > degrees + Constants.ANGLE_OFFSET){
            speed = -speedSupplier.getAsDouble();
        }
        lazySusan.spinSusan(speed * speedScalar);
        System.out.println("Susan Location: " + lazySusan.getLocation());
    }

}