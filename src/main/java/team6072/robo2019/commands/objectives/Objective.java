
package team6072.robo2019.commands.objectives;

import team6072.robo2019.subsystems.ElevatorSys;

public class Objective {



    // ---------------Rocket  Hatch---------

    private static final double ROCKET_HATCH_LO_INCHES = ((12 + 7) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    private static final int ROCKET_HATCH_LO = (int) (ROCKET_HATCH_LO_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_HATCH_MID_INCHES = (ROCKET_HATCH_LO_INCHES + 24 + 4);
    private static final int ROCKET_HATCH_MID = (int) (ROCKET_HATCH_MID_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_HATCH_HI_INCHES = (ROCKET_HATCH_MID_INCHES + 24 + 4);
    private static final int ROCKET_HATCH_HI = (int) (ROCKET_HATCH_HI_INCHES * ElevatorSys.TICKS_PER_INCH);

    // ---------------Rocket Cargo-------------

    private static final double ROCKET_CARGO_LO_INCHES = ((24 + 3.5) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    private static final int ROCKET_CARGO_LO = (int) (ROCKET_CARGO_LO_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_CARGO_MID_INCHES = (ROCKET_CARGO_LO_INCHES + 24 + 4);
    private static final int ROCKET_CARGO_MID = (int) (ROCKET_CARGO_MID_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_CARGO_HI_INCHES = (ROCKET_CARGO_MID_INCHES + 24 + 4);
    private static final int ROCKET_CARGO_HI = (int) (ROCKET_CARGO_HI_INCHES * ElevatorSys.TICKS_PER_INCH);

    // -----------------Cargoship  Hatch--------

    private static final double CARGOSHIP_HATCH_INCHES = ((12 + 7) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    private static final int CARGOSHIP_HATCH = (int) (CARGOSHIP_HATCH_INCHES * ElevatorSys.TICKS_PER_INCH);

    // ----------------CARGOSHIP CARGO---------

    private static final double CARGOSHIP_CARGO_INCHES = ((24 + 7.5 + 6.5 + 2) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    // extra 2 inches for safety^^^
    private static final int CARGOSHIP_CARGO = (int) (CARGOSHIP_CARGO_INCHES * ElevatorSys.TICKS_PER_INCH);

    public enum ElvTarget {
        RocketHatchHi(ROCKET_HATCH_HI), RocketHatchMid(ROCKET_HATCH_MID), RocketHatchLo(ROCKET_HATCH_LO),
        RocketCargoHi(ROCKET_CARGO_HI), RocketCargoMid(ROCKET_CARGO_MID), RocketCargoLo(ROCKET_CARGO_LO),
        CargoshipHatch(CARGOSHIP_HATCH), CargoshipCargo(CARGOSHIP_CARGO), HatchPickUp(CARGOSHIP_HATCH);

        private int mTicks;

        ElvTarget(int ticks) {
            mTicks = ticks;
        }

        public int getTicks() {
            return mTicks;
        }
    }

    private static final double RIGHT_BALL_ROCKET_ANGLE = 90;
    private static final double RIGHT_NEAR_HATCH_ROCKET_ANGLE = 45; // check whether or not that is accurate
    private static final double RIGHT_FAR_HATCH_ROCKET_ANGLE = 135; // check whether or not that is accurate
    private static final double LEFT_BALL_ROCKET_ANGLE = -90;
    private static final double LEFT_NEAR_HATCH_ROCKET_ANGLE = -45; // check whether or not that is accurate
    private static final double LEFT_FAR_HATCH_ROCKET_ANGLE = -135; // check whether or not that is accurate

    private final double ANGLED_TURN_TOLERANCE = 20.0;

    public enum TargetYaw {
        RIGHT_BALL(RIGHT_BALL_ROCKET_ANGLE), RIGHT_NEAR_HATCH(RIGHT_NEAR_HATCH_ROCKET_ANGLE),
        RIGHT_FAR_HATCH(RIGHT_FAR_HATCH_ROCKET_ANGLE), LEFT_BALL(LEFT_BALL_ROCKET_ANGLE),
        LEFT_NEAR_HATCH(LEFT_NEAR_HATCH_ROCKET_ANGLE), LEFT_FAR_HATCH(LEFT_FAR_HATCH_ROCKET_ANGLE), TEST(0);

        private double mAngle;

        public double getAngle() {
            return mAngle;
        }

        TargetYaw(double angle) {
            mAngle = angle;
        }
    }


    public enum Side {
        LeftNear, LeftMid, LeftFar, RightNear, RightMid, RightFar
    }



    private ElvTarget mElvTarget;
    private TargetYaw mTargetYaw;
    private Side mSide;

    // public Objective(ElvTarget elvTarget) {
    //     mElvTarget = elvTarget;
    //     mSide = null;
    //     mTargetYaw = null;
    // }

    // public Objective(TargetYaw targetAngle) {
    //     mElvTarget = null;
    //     mSide = side;
    //     mTargetYaw = targetAngle;
    // }

    public Objective(ElvTarget elvTarget, TargetYaw targetAngle) {
        mElvTarget = elvTarget;
        mTargetYaw = targetAngle;
    }


    @Override
    public String toString() {
        return String.format("Side: %s  Yaw: %s  Targ: %s", this.mSide, this.mTargetYaw, this.mElvTarget); 
    }


    public boolean isEqual(Objective otherObj) {
        return this.mElvTarget == otherObj.mElvTarget
                && this.mSide == otherObj.mSide
                && this.mTargetYaw == otherObj.mTargetYaw;
    }

    public ElvTarget getElvTarget() {
        return mElvTarget;
    }

    public TargetYaw getTargetYaw() {
        return mTargetYaw;
    }

    public Side getSide() {
        return mSide;
    }

}