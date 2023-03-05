package frc.robot.Subsystems;

import java.util.List;

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
        private DriveTrain driveTrain;
        private VertArm vertArm;
        private LazySusanSub lazySusan;
        private Pneumatics pneumatics;
        private static int locationChoice;
        private static int elementsChoice;
        private static int balanceChoice;
        private SequentialCommandGroup[] elementsChoiceGroupsArr;

        public enum Location {

                Left(0),
                Middle(1),
                Right(2);

                private Location(int location) {
                        locationChoice = location;
                }

                public static void setLocation(int location) {
                        locationChoice = location;
                }
        }

        public enum Elements {

                Zero(0),
                One(1),
                OneHold(2),
                Two(3),
                TwoHold(4);

                private Elements(int elements) {
                        elementsChoice = elements;
                }

                public static void setElements(int elements) {
                        elementsChoice = elements;
                }
        }

        public enum BalanceSP {

                No_Balance(0),
                Balance(1);

                private BalanceSP(int balance) {
                        balanceChoice = balance;
                }

                public static void setBalance(int bal) {
                        balanceChoice = bal;
                }
        }

        public SinglePaths(DriveTrain driveTrain, VertArm vertArm, LazySusanSub lazySusan, Pneumatics pneumatics) {
                this.driveTrain = driveTrain;
                this.vertArm = vertArm;
                this.lazySusan = lazySusan;
                this.pneumatics = pneumatics;
                this.elementsChoiceGroupsArr = new SequentialCommandGroup[] {
                                DriveCommunity(),
                                PlaceDrive(),
                                RunOneElement(),
                                RunTwoElement(),
                                RunThreeElement()
                };
        }

        private String[] driveToPieceOnePathName = { "LeftDriveToPieceOne", "MidDriveToPieceOne",
                        // Some are subbed out because they are the same path
                        "RightDriveToPieceOne" };
        private String[] driveToPieceTwoPathName = { "LeftDriveToPieceTwo", "MidDriveToPieceTwo",
                        "RightDriveToPieceTwo" };
        private boolean[] balanceBool = { false, true };
        private String[] driveToPlaceTwoPathName = { "LeftDriveToPlacementTwo", "MidDriveToPlacementTwo",
                        "RightDriveToPlacementTwo" };
        private String[] driveToBalanceOne = { "LeftPickupOneToBalance", "MidPickupOneToBalance",
                        "RightPickupOneToBalance" };
        private String[] driveToBalanceTwo = { "MidPickupOneToBalace", "MidPickupTwoToBalance",
                        "MidPickupTwoToBalance" };

        private SequentialCommandGroup PlaceFirstElementGroup() {
                return new SequentialCommandGroup(new PrintCommand("Place Element 1 Placement")
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
                return new SequentialCommandGroup(new PrintCommand("Place Element 2 Placement")
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
                return new SequentialCommandGroup(new PrintCommand("Prepare To Pickup Element")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                                .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
                                                Constants.SUSAN_180_ENC_POS)));
        }

        private SequentialCommandGroup PrepareToPlaceElement() {
                return new SequentialCommandGroup(new PrintCommand("Prepare To Place Element")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                                .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, 0)));
        }

        private SequentialCommandGroup PickupElement() {
                return new SequentialCommandGroup(new PrintCommand("Pickup Element")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_PICKUP_POS))
                                .andThen(new TogglePneumatics(pneumatics, false))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS)));
        }

        private SequentialCommandGroup balanceOne() {
                if (balanceBool[balanceChoice]) {
                        return new SequentialCommandGroup(new PrintCommand("Balance")
                                        .andThen(new ManualFollowAuto(driveTrain, driveToBalanceOne[locationChoice],
                                                        true))
                                        .andThen(new Balance(driveTrain)));
                } else {
                        return new SequentialCommandGroup(new PrintCommand("No Balance"));
                }
        }

        private SequentialCommandGroup balanceTwo() {
                if (balanceBool[balanceChoice]) {
                        return new SequentialCommandGroup(new PrintCommand("Balance")
                                        .andThen(new ManualFollowAuto(driveTrain, driveToBalanceTwo[locationChoice],
                                                        true))
                                        .andThen(new Balance(driveTrain)));
                } else {
                        return new SequentialCommandGroup(new PrintCommand("No Balance"));
                }
        }

        private SequentialCommandGroup DriveCommunity() {
                return new SequentialCommandGroup(new PrintCommand("Drive Community")
                                .andThen(new ManualFollowAuto(driveTrain, driveToPieceOnePathName[locationChoice],
                                                false)));
        }

        private SequentialCommandGroup PlaceDrive() {
                // return new SequentialCommandGroup(new PrintCommand("Just Drive"));
                return new SequentialCommandGroup(new PrintCommand("Place Drive")
                                .andThen(new ZeroVert(vertArm))
                                .andThen(new ZeroSusan(lazySusan))
                                .andThen(PlaceFirstElementGroup())
                                .andThen(new ManualFollowAuto(driveTrain,
                                                driveToPieceOnePathName[locationChoice], false))
                                .andThen(balanceOne()));
        }

        private SequentialCommandGroup RunOneElement() {
                // return new SequentialCommandGroup(new PrintCommand("One Element"));
                return new SequentialCommandGroup(new PrintCommand("Run One Element")
                                .andThen(new ZeroVert(vertArm))
                                .andThen(new ZeroSusan(lazySusan))
                                .andThen(PlaceFirstElementGroup())
                                .andThen(new ManualFollowAuto(driveTrain, driveToPieceOnePathName[locationChoice],
                                                false)
                                                .alongWith(PrepareToPickupElement()))
                                .andThen(PickupElement())
                                .andThen(balanceOne()));
        }

        private SequentialCommandGroup RunTwoElement() {
                // return new SequentialCommandGroup(new PrintCommand("Two Element"));
                return new SequentialCommandGroup(new PrintCommand("Run Two Element")
                                .andThen(new ZeroVert(vertArm))
                                .andThen(new ZeroSusan(lazySusan))
                                .andThen(PlaceFirstElementGroup())
                                .andThen(new ManualFollowAuto(driveTrain, driveToPieceOnePathName[locationChoice],
                                                false)
                                                .alongWith(PrepareToPickupElement()))
                                .andThen(PickupElement())
                                .andThen(new ManualFollowAuto(driveTrain, driveToPlaceTwoPathName[locationChoice], true)
                                                .alongWith(PrepareToPlaceElement()))
                                .andThen(PlaceSecondElementGroup())
                                .andThen(new ManualFollowAuto(driveTrain, driveToPieceTwoPathName[locationChoice],
                                                false))
                                .andThen(balanceTwo()));
        }

        private SequentialCommandGroup RunThreeElement() {
                // return new SequentialCommandGroup(new PrintCommand("Three Element"));
                return new SequentialCommandGroup(new PrintCommand("Run Three Element")
                                .andThen(new ZeroVert(vertArm))
                                .andThen(new ZeroSusan(lazySusan))
                                .andThen(PlaceFirstElementGroup())
                                .andThen(new ManualFollowAuto(driveTrain, driveToPieceOnePathName[locationChoice],
                                                false)
                                                .alongWith(PrepareToPickupElement()))
                                .andThen(PickupElement())
                                .andThen(new ManualFollowAuto(driveTrain, driveToPlaceTwoPathName[locationChoice], true)
                                                .alongWith(PrepareToPlaceElement()))
                                .andThen(PlaceSecondElementGroup())
                                .andThen(new ManualFollowAuto(driveTrain, driveToPieceTwoPathName[locationChoice],
                                                false)
                                                .alongWith(PrepareToPickupElement()))
                                .andThen(PickupElement())
                                .andThen(balanceTwo()));
        }

        public SequentialCommandGroup GetAutoCommand() {
                System.out.println(elementsChoice + "Number");
                return elementsChoiceGroupsArr[elementsChoice];
                // return RunOneElement();
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
