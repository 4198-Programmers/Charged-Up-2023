package frc.robot.Subsystems;

import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Commands.AutoReach;
import frc.robot.Commands.AutoRunIntake;
import frc.robot.Commands.AutoSusan;
import frc.robot.Commands.AutoVert;
import frc.robot.Commands.Balance;
import frc.robot.Commands.ManualFollowAuto;
import frc.robot.Commands.RunIntake;
import frc.robot.Commands.RunPathAuto;
import frc.robot.Commands.SlightTurnDrive;
import frc.robot.Commands.SlowManualFollowAuto;
import frc.robot.Commands.StopArm;
import frc.robot.Commands.TimedAuto;
import frc.robot.Commands.TogglePneumatics;
import frc.robot.Commands.SetRobotHeading;
import frc.robot.Commands.ZeroSusan;
import frc.robot.Commands.ZeroVert;
import frc.robot.Tags.CenterSusanPhoton;
import frc.robot.Tags.PhotonVision;

public class SinglePaths /* extends CommandBase */ {
        private DriveTrain driveTrain;
        private VertArm vertArm;
        private LazySusanSub lazySusan;
        private Pneumatics pneumatics;
        private Intake intake;
        private ReachArmSub reachArm;
        private PhotonVision vision;
        private static int locationChoice;
        private static int elementsChoice;
        private static int balanceChoice;
        private static int sideChoice;
        private static int autoChoice;
        private SequentialCommandGroup[] elementsChoiceGroupsArr;
        private SequentialCommandGroup[] autoChoiceGroupsArr;

        // public enum Location {

        // Left(0),
        // Right(1),
        // Middle(2);

        // private Location(int location) {
        // locationChoice = location;
        // }

        // public static void setLocation(int location) {
        // locationChoice = location;
        // }

        // public static int getLocationChoice() {
        // return locationChoice;
        // }
        // }

        // public enum Elements {

        // Zero(0),
        // One(1),
        // OneHold(2),
        // Two(3),
        // TwoHold(4),
        // justPlace(5),
        // ChargeStation(6),
        // PlaceDriveCharge(7),
        // PlaceDriveChargeMid(8);

        // private Elements(int elements) {
        // elementsChoice = elements;
        // }

        //
        // }

        // public static void setElements(int elements) {
        // elementsChoice = elements;
        // }

        // public enum BalanceSP {

        // No_Balance(0),
        // Balance(1);

        // private BalanceSP(int balance) {
        // balanceChoice = balance;
        // }

        // public static void setBalance(int bal) {
        // balanceChoice = bal;
        // }
        // }

        // public enum SideChoice {
        // Red(0),
        // Blue(1);

        // private SideChoice(int Side) {
        // sideChoice = Side;
        // }

        // public static void setSide(int side) {
        // sideChoice = side;
        // }
        // }

        public SinglePaths(DriveTrain driveTrain, VertArm vertArm, LazySusanSub lazySusan, Pneumatics pneumatics,
                        Intake intake, ReachArmSub reachArm, PhotonVision vision) {
                this.driveTrain = driveTrain;
                this.vertArm = vertArm;
                this.lazySusan = lazySusan;
                this.pneumatics = pneumatics;
                this.intake = intake;
                this.reachArm = reachArm;
                this.vision = vision;
                // this.elementsChoiceGroupsArr = new SequentialCommandGroup[] {
                // // DriveCommunity(),
                // // PlaceDrive(),
                // // RunOneElement(),
                // // RunTwoElement(),
                // // RunThreeElement(),
                // justPlace(),
                // // PlaceChargeStation(),
                // PlaceDriveCharge(),
                // PlaceDriveChargeMid()
                // };
                this.autoChoiceGroupsArr = new SequentialCommandGroup[] {
                                PlaceDriveChargeMid(),
                                PlaceDriveChargeLeftNoCable(), // Left No Cable
                                JustPlaceRight(),
                                PlaceDriveChargeRightNoCable(),
                                PlaceDriveChargeRightCable(), // right and left are driver based
                                PlaceDriveChargeLeftCable(),
                                PlaceDriveNew(),
                                JustDrive(),
                                PlaceCharge(),
                                JustPlaceLeft() // driver oriented
                };
        }

        public void setAutoChoice(int choice) {
                autoChoice = choice;
        }

        // private String[] driveToPieceTwoPathName = { "LeftDriveToPieceTwo",
        // "RightDriveToPieceTwo",
        // "MidDriveToPieceTwo", };
        // private String[] driveStraight = { "StraightToPieceLeft",
        // "StraightToPieceRight", "StraightToPieceMid", };
        // private boolean[] balanceBool = { false, true };
        // private String[] driveToPlaceTwoPathName = { "LeftDriveToPlacementTwo",
        // "RightDriveToPlacementTwo",
        // "MidDriveToPlacementTwo", };
        // private String[] driveToBalanceOne = { "LeftPickupOneToBalance",
        // "RightPickupOneToBalance",
        // "MidPickupOneToBalance", };
        // private String[] driveToBalanceTwo = { "MidPickupOneToBalace",
        // "MidPickupTwoToBalance",
        // "MidPickupTwoToBalance" }; // neg cc susan
        // Some are subbed out because they are the same path

        // private String[] driveToPieceOneNameBlue = { "ClearDriveToPieceOne", // Left
        // is 0
        // "CableDriveToPieceOne" };
        // private String[] driveToPieceOneNameRed = { "CableDriveToPieceOne", // Left
        // is 0
        // "ClearDriveToPieceOne" };

        // private double[] sideToPlace = { Constants.RIGHT_TOP_PLACEMENT_SUSAN,
        // Constants.LEFT_TOP_PLACEMENT_SUSAN };

        // private String drivePieceOne(int location, int side) {
        // String mid = "MidDriveToPieceOne";
        // if (location != 2) {
        // if (side == 0) {
        // return driveToPieceOneNameRed[location];
        // } else if (side == 1) {
        // return driveToPieceOneNameBlue[location];
        // }
        // }
        // return mid;
        // }

        private double safeSpinSpeed(int location, int side) {
                double[] sideSafeSpinRight = { -Constants.AUTO_SUSAN_SPEED, Constants.AUTO_SUSAN_SPEED };
                double[] sideSafeSpinLeft = { Constants.AUTO_SUSAN_SPEED, -Constants.AUTO_SUSAN_SPEED };
                if (location == 0) {
                        return sideSafeSpinLeft[side];
                } else if (location == 1 || location == 2) {
                        return sideSafeSpinRight[side];
                } else {
                        return 0;
                }

        }

        private SequentialCommandGroup PlaceTopRightElementGroup() {
                return new SequentialCommandGroup(new PrintCommand("Place Element 1 Placement")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT)
                                                .alongWith(new AutoReach(reachArm,
                                                                Constants.AUTO_REACH_SPEED,
                                                                Constants.TOP_REACH_RIGHT_PLACEMENT))
                                                .alongWith(new AutoRunIntake(intake, Constants.INTAKE_IN_SPEED))
                                                .alongWith(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
                                                                Constants.RIGHT_TOP_PLACEMENT_SUSAN)))
                                .andThen(new WaitCommand(0.5))
                                .andThen(new AutoRunIntake(intake, Constants.INTAKE_OUT_SPEED))
                                .andThen(new AutoReach(reachArm, Constants.AUTO_REACH_SPEED, 0))
                                .andThen(new PrintCommand("Done Placing")));
        }

        private SequentialCommandGroup PlaceTopLeftElementGroup() {
                return new SequentialCommandGroup(new PrintCommand("Place Element 1 Placement")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT)
                                                .alongWith(new AutoReach(reachArm,
                                                                Constants.AUTO_REACH_SPEED,
                                                                Constants.TOP_REACH_LEFT_PLACEMENT))
                                                .alongWith(new AutoRunIntake(intake, Constants.INTAKE_IN_SPEED))
                                                .alongWith(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
                                                                Constants.LEFT_TOP_PLACEMENT_SUSAN)))
                                .andThen(new WaitCommand(0.5))
                                .andThen(new AutoRunIntake(intake, Constants.INTAKE_OUT_SPEED))
                                .andThen(new AutoReach(reachArm, Constants.AUTO_REACH_SPEED, 0))
                                .andThen(new PrintCommand("Done Placing")));
        }

        private SequentialCommandGroup JustPlaceRight() {
                return new SequentialCommandGroup(PlaceTopRightElementGroup());
        }

        private SequentialCommandGroup JustPlaceLeft() {
                return new SequentialCommandGroup(PlaceTopLeftElementGroup());
        }

        // private SequentialCommandGroup PlaceLeftElementApril() {
        // return new SequentialCommandGroup(
        // new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VISIBLE_TAG_VERT_ENC)
        // .andThen(new CenterSusanPhoton(vision, driveTrain, lazySusan, 0, 0,
        // Constants.DISTANCE_FROM_TAG))
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VERT_MID_SHELF_PLACEMENT_ENC_MID)
        // .alongWith(new AutoReach(reachArm,
        // Constants.AUTO_REACH_SPEED, 5450))
        // .alongWith(new AutoRunIntake(intake,
        // Constants.INTAKE_IN_SPEED)))
        // .andThen(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED,
        // Constants.SUSAN_LEFT_FROM_TAG_ENC))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new AutoRunIntake(intake, Constants.INTAKE_OUT_SPEED))
        // .andThen(new AutoReach(reachArm, Constants.AUTO_REACH_SPEED, 0)));
        // }

        // private SequentialCommandGroup PlaceSecondElementGroup() { // places on right
        // DOESNT WORK
        // return new SequentialCommandGroup(new PrintCommand("Place Element 2
        // Placement")
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.VERT_SAFE_TO_SPIN_ENC_POS))
        // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
        // Constants.PLACE_TOP_VERT)
        // .alongWith(new AutoReach(reachArm,
        // Constants.AUTO_REACH_SPEED,
        // Constants.TOP_REACH_PLACEMENT))
        // .alongWith(new AutoRunIntake(intake, Constants.INTAKE_IN_SPEED)))
        // .andThen(new ManualFollowAuto(driveTrain, "ShortForward", true))
        // .andThen(new AutoSusan(lazySusan, safeSpinSpeed(locationChoice, sideChoice),
        // Constants.LEFT_TOP_PLACEMENT_SUSAN)
        // .alongWith(new AutoRunIntake(intake, Constants.INTAKE_IN_SPEED)))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new AutoRunIntake(intake, Constants.INTAKE_OUT_SPEED))
        // .andThen(new AutoReach(reachArm, Constants.AUTO_REACH_SPEED, 0))
        // .andThen(new PrintCommand("Done Placing")));
        // }

        private SequentialCommandGroup PrepareToPickupElement() {
                return new SequentialCommandGroup(new PrintCommand("Prepare To Pickup Element")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                                .andThen(new AutoSusan(lazySusan, safeSpinSpeed(locationChoice, sideChoice),
                                                Constants.SUSAN_180_ENC_POS)
                                                .alongWith(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS))));
        }

        private SequentialCommandGroup PrepareToPlaceElement() {
                return new SequentialCommandGroup(new PrintCommand("Prepare To Place Element")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                                .andThen(new AutoSusan(lazySusan, safeSpinSpeed(locationChoice, sideChoice), 0)
                                                .alongWith(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS))));
        }

        private SequentialCommandGroup PickupElement() {
                return new SequentialCommandGroup(new PrintCommand("Pickup Element")
                                .andThen(new AutoReach(reachArm, Constants.AUTO_REACH_SPEED,
                                                750))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, Constants.VERT_PICKUP_POS)
                                                .alongWith(new AutoRunIntake(intake, Constants.INTAKE_IN_SPEED)))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS)
                                                .alongWith(new RunIntake(intake, Constants.INTAKE_IN_SPEED)))
                                .andThen(new AutoReach(reachArm, Constants.AUTO_REACH_SPEED, 0)));
        }

        // private SequentialCommandGroup balanceOne() {
        // if (balanceBool[balanceChoice]) {
        // return new SequentialCommandGroup(new PrintCommand("Balance")
        // .andThen(new ManualFollowAuto(driveTrain, driveToBalanceOne[locationChoice],
        // true))
        // .andThen(new Balance(driveTrain)));
        // } else {
        // return new SequentialCommandGroup(new PrintCommand("No Balance"));
        // }
        // }

        // private SequentialCommandGroup balanceTwo() {
        // if (balanceBool[balanceChoice]) {
        // return new SequentialCommandGroup(new PrintCommand("Balance")
        // .andThen(new ManualFollowAuto(driveTrain, driveToBalanceTwo[locationChoice],
        // true))
        // .andThen(new Balance(driveTrain)));
        // } else {
        // return new SequentialCommandGroup(new PrintCommand("No Balance"));
        // }
        // }

        // private SequentialCommandGroup PlaceChargeStation() {
        // return new SequentialCommandGroup(PlaceTopRightElementGroup()
        // .andThen(new SlowManualFollowAuto(driveTrain, "OnCharge", false))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new SlowManualFollowAuto(driveTrain, "OffCharge", false))
        // .andThen(new WaitCommand(0.5))
        // .andThen(new SlowManualFollowAuto(driveTrain, "BackToCharge", true))
        // .andThen(new SlightTurnDrive(driveTrain)));
        // }

        // private SequentialCommandGroup DriveCommunity() {
        // return new SequentialCommandGroup(new PrintCommand("Drive Community")
        // .andThen(new ManualFollowAuto(driveTrain, driveStraight[locationChoice],
        // false)));
        // }

        // private SequentialCommandGroup PlaceDrive() {
        // // return new SequentialCommandGroup(new PrintCommand("Just Drive"));
        // return new SequentialCommandGroup(new PrintCommand("Place Drive")
        // .andThen(new ZeroVert(vertArm))
        // .andThen(new ZeroSusan(lazySusan))
        // .andThen(PlaceTopRightElementGroup())
        // .andThen(new ManualFollowAuto(driveTrain,
        // drivePieceOne(locationChoice, sideChoice), false))
        // .andThen(balanceOne())
        // .andThen(new SlightTurnDrive(driveTrain)));
        // }

        private SequentialCommandGroup PlaceDriveChargeLeftNoCable() { // Left No Cable
                return new SequentialCommandGroup(new PrintCommand("Place Drive Charge")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT).raceWith(new WaitCommand(2)))
                                .andThen(new TimedAuto(driveTrain, 750, 0, -0.75, 0, 0))
                                .andThen(new TimedAuto(driveTrain, 250, 1, 0, 0, 0))
                                .andThen(PlaceTopRightElementGroup())
                                .andThen(new TimedAuto(driveTrain, 3000, 0, 2, 0, 0))
                                .andThen((new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS) // pulls arm down to balance easier
                                                .alongWith(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, 0)))
                                                // spins susan so arm can sit in robot
                                                .raceWith(new TimedAuto(driveTrain, 750, -2.5, 0, 0, 0)))
                                .andThen(new SetRobotHeading(driveTrain, 0).raceWith(new WaitCommand(1)))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, 0) // move arm in to safety
                                                .raceWith(new TimedAuto(driveTrain, 1500, 0, -1.25, 0, 0)))
                                // drives onto charge station
                                .andThen(new Balance(driveTrain))
                                .andThen(new SlightTurnDrive(driveTrain)));

        }

        private SequentialCommandGroup PlaceCharge() { // works within 15 seconds
                return new SequentialCommandGroup(new PrintCommand("Place Charge")
                                .andThen(new ZeroSusan(lazySusan).alongWith(new ZeroVert(vertArm)))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT)
                                                .raceWith(new WaitCommand(2)))
                                .andThen(new TimedAuto(driveTrain, 750, 0, -0.75, 0, 0))
                                .andThen(new TimedAuto(driveTrain, 250, 1, 0, 0, 0))
                                .andThen(PlaceTopRightElementGroup())
                                .andThen(new TimedAuto(driveTrain, 1000, 0, 1.25, 0, 0))
                                .andThen(new TimedAuto(driveTrain, 1950, 0, 1.25, 0, 0)
                                                .alongWith(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                                                .alongWith(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, 0)))
                                .andThen(new Balance(driveTrain)
                                                .alongWith(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                                0)))
                                .andThen(new SlightTurnDrive(driveTrain)));

        }

        private SequentialCommandGroup PlaceDriveChargeRightNoCable() { // speedy one
                return new SequentialCommandGroup(new PrintCommand("Charge Right")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT)
                                                .raceWith(new WaitCommand(2)))
                                .andThen(new TimedAuto(driveTrain, 750, 0, -0.75, 0, 0)) // drive into placement
                                .andThen(new TimedAuto(driveTrain, 250, 1, 0, 0, 0)) // drive right to placement
                                .andThen(PlaceTopLeftElementGroup()) // place element
                                .andThen(new TimedAuto(driveTrain, 2000, 0, 2.5, 0, 0)) // drive out past station
                                .andThen((new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS) // pulls arm down to balance easier
                                                .alongWith(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, 0)))
                                                // spins susan so arm can sit in robot
                                                .raceWith(new TimedAuto(driveTrain, 750, -2.5, 0, 0, 0)))
                                // drives sideways to chargestation
                                .andThen(new SetRobotHeading(driveTrain, 0).raceWith(new WaitCommand(1)))
                                // spins the robot to 0 so that it goes straight on the station
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, 0) // move arm in to safety
                                                .raceWith(new TimedAuto(driveTrain, 1500, 0, -1.25, 0, 0)))
                                // drives onto charge station
                                .andThen(new Balance(driveTrain)) // balances charge station
                                .andThen(new SlightTurnDrive(driveTrain))); // locks wheels

        }

        private SequentialCommandGroup PlaceDriveChargeLeftCable() {
                return new SequentialCommandGroup(new PrintCommand("Charge Left")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT).raceWith(new WaitCommand(2)))
                                .andThen(new TimedAuto(driveTrain, 750, 0, -0.75, 0, 0))
                                .andThen(new TimedAuto(driveTrain, 250, 1, 0, 0, 0))
                                .andThen(PlaceTopRightElementGroup())
                                .andThen(new TimedAuto(driveTrain, 3500, 0, 1.5, 0, 0))
                                .andThen((new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS) // pulls arm down to balance easier
                                                .alongWith(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, 0)))
                                                // spins susan so arm can sit in robot
                                                .raceWith(new TimedAuto(driveTrain, 750, -2.5, 0, 0, 0)))
                                .andThen(new SetRobotHeading(driveTrain, 0).raceWith(new WaitCommand(1)))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, 0) // move arm in to safety
                                                .raceWith(new TimedAuto(driveTrain, 1500, 0, -1.25, 0, 0)))
                                // drives onto charge station
                                .andThen(new Balance(driveTrain))
                                .andThen(new SlightTurnDrive(driveTrain)));

        }

        private SequentialCommandGroup PlaceDriveChargeRightCable() {
                return new SequentialCommandGroup(new PrintCommand("Charge Right")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT).raceWith(new WaitCommand(2)))
                                .andThen(new TimedAuto(driveTrain, 750, 0, -0.75, 0, 0))
                                .andThen(new TimedAuto(driveTrain, 250, 1, 0, 0, 0))
                                .andThen(PlaceTopLeftElementGroup())
                                .andThen(new TimedAuto(driveTrain, 3500, 0, 1.5, 0, 0))
                                .andThen((new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS) // pulls arm down to balance easier
                                                .alongWith(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, 0)))
                                                // spins susan so arm can sit in robot
                                                .raceWith(new TimedAuto(driveTrain, 750, -2.5, 0, 0, 0)))
                                .andThen(new SetRobotHeading(driveTrain, 0).raceWith(new WaitCommand(1)))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, 0) // move arm in to safety
                                                .raceWith(new TimedAuto(driveTrain, 1500, 0, -1.25, 0, 0)))
                                // drives onto charge station
                                .andThen(new Balance(driveTrain))
                                .andThen(new SlightTurnDrive(driveTrain)));

        }

        private SequentialCommandGroup PlaceDriveChargeMid() { // Mid
                return new SequentialCommandGroup(new PrintCommand("Charge Mid")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT).raceWith(new WaitCommand(2)))
                                .andThen(new TimedAuto(driveTrain, 750, 0, -0.75, 0, 0))
                                .andThen(new TimedAuto(driveTrain, 250, 1, 0, 0, 0))
                                .andThen(PlaceTopRightElementGroup())
                                .andThen(new TimedAuto(driveTrain, 25, 0, 0, -2, 0))
                                .andThen(new TimedAuto(driveTrain, 2500, 0, 1.1, 0, 0))
                                .andThen(new TimedAuto(driveTrain, 2500, 0, 1.1, 0, 0)
                                                .alongWith(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                                Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                                                .alongWith(new AutoSusan(lazySusan, Constants.AUTO_SUSAN_SPEED, 0)))
                                .andThen(new SetRobotHeading(driveTrain, 0).raceWith(new WaitCommand(1)))
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED, 0)
                                                .raceWith(new TimedAuto(driveTrain, 2950, 0, -1.1, 0, 0)))
                                .andThen(new Balance(driveTrain))
                                .andThen(new SlightTurnDrive(driveTrain)));

        }

        private SequentialCommandGroup PlaceDriveNew() {
                return new SequentialCommandGroup(new PrintCommand("Place Drive")
                                .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                                                Constants.PLACE_TOP_VERT).raceWith(new WaitCommand(2)))
                                .andThen(new TimedAuto(driveTrain, 750, 0, -0.75, 0, 0))
                                .andThen(new TimedAuto(driveTrain, 250, 1, 0, 0, 0))
                                .andThen(PlaceTopRightElementGroup())
                                .andThen(new TimedAuto(driveTrain, 2000, 0, 1.5, 0, 0))
                                .andThen(new TimedAuto(driveTrain, 2000, 0, 1.5, 0, 0)));
        }

        private SequentialCommandGroup JustDrive() {
                return new SequentialCommandGroup(new PrintCommand("Just Drive")
                                .andThen(new TimedAuto(driveTrain, 4000, 0, 1.5, 0, 0)));
        }

        // private SequentialCommandGroup RunOneElement() {
        // // return new SequentialCommandGroup(new PrintCommand("One Element"));
        // return new SequentialCommandGroup(new PrintCommand("Run One Element")
        // .andThen(new ZeroVert(vertArm))
        // .andThen(new ZeroSusan(lazySusan))
        // .andThen(PlaceTopRightElementGroup())
        // .andThen(new ManualFollowAuto(driveTrain, drivePieceOne(locationChoice,
        // sideChoice),
        // false)
        // .alongWith(PrepareToPickupElement()))
        // .andThen(PickupElement())
        // .andThen(balanceOne())
        // .andThen(new SlightTurnDrive(driveTrain)));
        // }

        // private SequentialCommandGroup RunTwoElement() {
        // // return new SequentialCommandGroup(new PrintCommand("Two Element"));
        // return new SequentialCommandGroup(new PrintCommand("Run Two Element")
        // .andThen(new ZeroVert(vertArm))
        // .andThen(new ZeroSusan(lazySusan))
        // .andThen(PlaceTopRightElementGroup())
        // .andThen(new ManualFollowAuto(driveTrain, drivePieceOne(locationChoice,
        // sideChoice),
        // false)
        // .alongWith(PrepareToPickupElement()))
        // .andThen(PickupElement())
        // .andThen(new ManualFollowAuto(driveTrain,
        // driveToPlaceTwoPathName[locationChoice], true)
        // .alongWith(PrepareToPlaceElement()))
        // .andThen(PlaceSecondElementGroup())
        // .andThen(new ManualFollowAuto(driveTrain,
        // driveToPieceTwoPathName[locationChoice],
        // false))
        // .andThen(balanceTwo())
        // .andThen(new SlightTurnDrive(driveTrain)));
        // }

        // private SequentialCommandGroup RunThreeElement() {
        // // return new SequentialCommandGroup(new PrintCommand("Three Element"));
        // return new SequentialCommandGroup(new PrintCommand("Run Three Element")
        // .andThen(new ZeroVert(vertArm))
        // .andThen(new ZeroSusan(lazySusan))
        // .andThen(PlaceTopRightElementGroup())
        // .andThen(new ManualFollowAuto(driveTrain, drivePieceOne(locationChoice,
        // sideChoice),
        // false)
        // .alongWith(PrepareToPickupElement()))
        // .andThen(PickupElement())
        // .andThen(new ManualFollowAuto(driveTrain,
        // driveToPlaceTwoPathName[locationChoice], true)
        // .alongWith(PrepareToPlaceElement()))
        // .andThen(PlaceSecondElementGroup())
        // .andThen(new ManualFollowAuto(driveTrain,
        // driveToPieceTwoPathName[locationChoice],
        // false)
        // .alongWith(PrepareToPickupElement()))
        // .andThen(PickupElement())
        // .andThen(balanceTwo())
        // .andThen(new SlightTurnDrive(driveTrain)));
        // }

        public SequentialCommandGroup GetAutoCommand() {
                return autoChoiceGroupsArr[autoChoice];
                // return DriveCommunity();
                // return RunOneElement();
                // return new SequentialCommandGroup(new ZeroVert(vertArm)
                // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                // Constants.VERT_SAFE_TO_SPIN_ENC_POS))
                // .andThen(new AutoSusan(lazySusan, safeSpinSpeed(),
                // Constants.LEFT_PLACEMENT_ENC_POS))
                // .andThen(new AutoVert(vertArm, Constants.AUTO_VERT_SPEED,
                // Constants.VERT_BOTTOM_SHELF_PLACEMENT_ENC_SIDES))
                // .andThen(new TogglePneumatics(pneumatics, true))
                // .andThen(new ManualFollowAuto(driveTrain, "StraightToPiece")));

        }

}
