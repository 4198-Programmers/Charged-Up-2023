package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.AutoSusan;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.Balance;
import frc.robot.Commands.ManualFollowAuto;
import frc.robot.Commands.RunPathAuto;
import frc.robot.Commands.StopArm;
import frc.robot.Commands.TogglePneumatics;
import frc.robot.Commands.ZeroSusan;
import frc.robot.Commands.ZeroVert;

public class SinglePaths /* extends CommandBase */ {
    private String fieldChoice;
    private boolean balance;
    private DriveTrain driveTrain;
    private VertArm vertArm;
    private LazySusanSub lazySusan;
    private Pneumatics pneumatics;
    private static int locationChoice;
    private static int elementsChoice;
    private static int balanceChoice;

    public enum Location {

        Left(0),
        Middle(1),
        Right(2);

        private Location(int location) {
            locationChoice = location;
        }
    }

    public enum Elements {

        Zero(0),
        One(1),
        Two(2),
        Three(3);

        private Elements(int elements) {
            elementsChoice = elements;
        }
    }

    public enum BalanceSP {

        No_Balance(0),
        Balance(1);

        private BalanceSP(int balance) {
            balanceChoice = balance;
        }
    }

    public SinglePaths(DriveTrain driveTrain, VertArm vertArm, LazySusanSub lazySusan, Pneumatics pneumatics) {
        this.driveTrain = driveTrain;
        this.vertArm = vertArm;
        this.lazySusan = lazySusan;
        this.pneumatics = pneumatics;
        // addRequirements(driveTrain, vertArm, lazySusan, pneumatics);
        this.elementsChoiceGroupsArr = new SequentialCommandGroup[] {
                JustDrive(),
                RunOneElement(),
                RunTwoElement(),
                RunThreeElement()
        };
    }

    private String[] driveToPieceOnePathNameArr = { "LeftDriveToPieceOne", "MidDriveToPieceOne",
            "RightDriveToPieceOne" };
    private String[] driveToPieceTwoPathName = { "LeftDriveToPieceTwo", "MidDriveToPieceTwo",
            "RightDriveToPieceTwo" };
    private SequentialCommandGroup[] elementsChoiceGroupsArr;
    private boolean[] balanceBool = { false, true };
    private String[] driveToPlaceTwoPathName = { "LeftDriveToPlacementTwo", "MiddleDriveToPlacementTwo",
            "RightDriveToPlacementTwo" };
    private String[] driveToBalanceOne = { "LeftPickupOneToBalance", "MidPickupOneToBalance",
            "RightPickupOneToBalance" };
    private String[] driveToBalanceTwo = { "LeftPickupTwoToBalance", "MidPickupTwoToBalance",
            "RightPickupTwoToBalance" };

    private SequentialCommandGroup PlaceFirstElementGroup() {
        // addRequirements(vertArm, lazySusan, pneumatics);
        return new SequentialCommandGroup(new PrintCommand("Prep Element Placement")
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
                        Constants.LEFT_PLACEMENT_ENC_POS))
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_BOTTOM_SHELF_PLACEMENT_ENC_SIDES))
                .andThen(new TogglePneumatics(pneumatics, true))
                .andThen(new PrintCommand("Done Placing")));
    }

    private SequentialCommandGroup PlaceSecondElementGroup() {
        // addRequirements(vertArm, lazySusan, pneumatics);
        return new SequentialCommandGroup(new PrintCommand("Prep Element Placement")
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
                        Constants.RIGHT_PLACEMENT_ENC_POS))
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                        Constants.VERT_BOTTOM_SHELF_PLACEMENT_ENC_SIDES))
                .andThen(new TogglePneumatics(pneumatics, true))
                .andThen(new PrintCommand("Done Placing")));
    }

    private SequentialCommandGroup PrepareToPickupElement() {
        // addRequirements(vertArm, lazySusan);
        return new SequentialCommandGroup(new PrintCommand("Prepare To Pickup Element")
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, Constants.SUSAN_180_ENC_POS)));
    }

    private SequentialCommandGroup PrepareToPlaceElement() {
        // addRequirements(vertArm, lazySusan);
        return new SequentialCommandGroup(new PrintCommand("Prepare To Place Element")
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, 0)));
    }

    private SequentialCommandGroup PickupElement() {
        // addRequirements(vertArm, pneumatics);
        return new SequentialCommandGroup(new PrintCommand("Pickup Element")
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_PICKUP_POS))
                .andThen(new TogglePneumatics(pneumatics, true))
                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_SAFE_TO_SPIN_ENC_POS)));
    }

    private SequentialCommandGroup balance() {
        // addRequirements(driveTrain);
        if (balanceBool[balanceChoice]) {
            return new SequentialCommandGroup(
                    new ManualFollowAuto(driveTrain, driveToBalanceOne[locationChoice])
                            .andThen(new Balance(driveTrain)));
        } else {
            return new SequentialCommandGroup(new PrintCommand("No Balance"));
        }
    }

    private SequentialCommandGroup JustDrive() {
        return new SequentialCommandGroup(new PrintCommand("Just Drive"));
        // return new SequentialCommandGroup(new ManualFollowAuto(driveTrain,
        // driveToPiecePathNameArr[locationChoice]));
    }

    private SequentialCommandGroup RunOneElement() {
        // return new SequentialCommandGroup(new PrintCommand("One Element"));
        return new SequentialCommandGroup(new ZeroVert(vertArm)
                .andThen(new ZeroSusan(lazySusan))
                .andThen(PlaceFirstElementGroup())
                .andThen(new ManualFollowAuto(driveTrain, driveToPieceOnePathNameArr[locationChoice])
                        .alongWith(PrepareToPickupElement()))
                .andThen(PickupElement())
                .andThen(balance()));
    }

    private SequentialCommandGroup RunTwoElement() {
        // return new SequentialCommandGroup(new PrintCommand("Two Element"));
        // addRequirements(driveTrain, vertArm, lazySusan, pneumatics);
        return new SequentialCommandGroup(PlaceFirstElementGroup()
                .andThen(new ManualFollowAuto(driveTrain, driveToPieceOnePathNameArr[locationChoice])
                        .alongWith(PrepareToPickupElement()))
                .andThen(PickupElement())
                .andThen(new ManualFollowAuto(driveTrain, driveToPlaceTwoPathName[locationChoice])
                        .alongWith(PrepareToPlaceElement()))
                .andThen(PlaceSecondElementGroup())
                .andThen(new ManualFollowAuto(driveTrain, driveToPieceTwoPathName[locationChoice])));
    }

    private SequentialCommandGroup RunThreeElement() {
        return new SequentialCommandGroup(new PrintCommand("Three Element"));
    }

    public SequentialCommandGroup GetAutoCommand() {
        // return elementsChoiceGroupsArr[elementsChoice];
        return RunOneElement();
        // return new SequentialCommandGroup(new ZeroVert(vertArm)
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VERT_SAFE_TO_SPIN_ENC_POS))
        // .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
        // Constants.LEFT_PLACEMENT_ENC_POS))
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VERT_BOTTOM_SHELF_PLACEMENT_ENC_SIDES))
        // .andThen(new TogglePneumatics(pneumatics, true))
        // .andThen(new ManualFollowAuto(driveTrain, "StraightToPiece")));

    }

    // public SequentialCommandGroup GetAutoCommand(){
    // return elementsChoiceGroupsArr[elementsChoice];
    // }

}
