package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.LazySusanSub;
import frc.robot.Subsystems.Pneumatics;
import frc.robot.Subsystems.ReachArmSub;
import frc.robot.Subsystems.VertArm;

//All auto is bot oriented
public class AutoContainer {
    public enum LocationAndNumberOfElements{
        //Location and #Of Elements
        OneAndOneE(new SequentialCommandGroup()),
        OneAndOneB(new SequentialCommandGroup()),
        OneAndTwoE(new SequentialCommandGroup()),
        OneAndTwoB(new SequentialCommandGroup()),
        OneAndThreeE(new SequentialCommandGroup()),
        OneAndThreeB(new SequentialCommandGroup()),
        TwoAndOneB(new SequentialCommandGroup()),
        TwoAndOneE(new SequentialCommandGroup()),
        TwoAndTwoE(new SequentialCommandGroup()),
        TwoAndTwoB(new SequentialCommandGroup()),
        TwoAndThreeE(new SequentialCommandGroup()),
        TwoAndThreeB(new SequentialCommandGroup()),
        ThreeAndOneB(new SequentialCommandGroup()),
        ThreeAndOneE(new SequentialCommandGroup()),
        ThreeAndTwoE(new SequentialCommandGroup()),
        ThreeAndTwoB(new SequentialCommandGroup()),
        ThreeAndThreeE(new SequentialCommandGroup()),
        ThreeAndThreeB(new SequentialCommandGroup());
        
        private SequentialCommandGroup sequentialCommandGroup;
    private LocationAndNumberOfElements(SequentialCommandGroup sequentialCommandGroup){
        this.sequentialCommandGroup = sequentialCommandGroup;
    }
    public SequentialCommandGroup getCommand(){
        return sequentialCommandGroup;
    }
    }
    DriveTrain driveTrain;
    LazySusanSub lazySusanSub;
    Pneumatics pneumatics;
    ReachArmSub reachArmSub;
    VertArm vertArm;

    public AutoContainer(DriveTrain driveTrain, LazySusanSub lazySusanSub, Pneumatics pneumatics,
            ReachArmSub reachArmSub, VertArm vertArm) {
        this.driveTrain = driveTrain;
        this.lazySusanSub = lazySusanSub;
        this.pneumatics = pneumatics;
        this.reachArmSub = reachArmSub;
        this.vertArm = vertArm;
    }
}
