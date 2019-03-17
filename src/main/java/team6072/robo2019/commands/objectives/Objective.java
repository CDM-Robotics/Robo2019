
package team6072.robo2019.commands.objectives;

import team6072.robo2019.subsystems.ElevatorSys;

public class Objective{
    
    
    public enum Side {
        Left(),
        Right()
    }

// --------------------------------------Rocket
    // Hatch----------------------------------------------

    private static final double ROCKET_HATCH_LO_INCHES = ((12 + 7) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    private static final int ROCKET_HATCH_LO = (int) (ROCKET_HATCH_LO_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_HATCH_MID_INCHES = (ROCKET_HATCH_LO_INCHES + 24 + 4);
    private static final int ROCKET_HATCH_MID = (int) (ROCKET_HATCH_MID_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_HATCH_HI_INCHES = (ROCKET_HATCH_MID_INCHES + 24 + 4);
    private static final int ROCKET_HATCH_HI = (int) (ROCKET_HATCH_HI_INCHES * ElevatorSys.TICKS_PER_INCH);

    // -------------------------------------Rocket
    // Cargo----------------------------------------------

    private static final double ROCKET_CARGO_LO_INCHES = ((24 + 3.5) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    private static final int ROCKET_CARGO_LO = (int) (ROCKET_CARGO_LO_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_CARGO_MID_INCHES = (ROCKET_CARGO_LO_INCHES + 24 + 4);
    private static final int ROCKET_CARGO_MID = (int) (ROCKET_CARGO_MID_INCHES * ElevatorSys.TICKS_PER_INCH);

    private static final double ROCKET_CARGO_HI_INCHES = (ROCKET_CARGO_MID_INCHES + 24 + 4);
    private static final int ROCKET_CARGO_HI = (int) (ROCKET_CARGO_HI_INCHES * ElevatorSys.TICKS_PER_INCH);

    // --------------------------------------Cargoship
    // Hatch----------------------------------------

    private static final double CARGOSHIP_HATCH_INCHES = ((12 + 7) - ElevatorSys.ELEVATOR_FLOOR_INCHES);
    private static final int CARGOSHIP_HATCH = (int) (CARGOSHIP_HATCH_INCHES * ElevatorSys.TICKS_PER_INCH);

    // --------------------------------------CARGOSHIP
    // CARGO----------------------------------------

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

    public enum TargetAngle {
        RIGHT_BALL(RIGHT_BALL_ROCKET_ANGLE), RIGHT_NEAR_HATCH(RIGHT_NEAR_HATCH_ROCKET_ANGLE),
        RIGHT_FAR_HATCH(RIGHT_FAR_HATCH_ROCKET_ANGLE), LEFT_BALL(LEFT_BALL_ROCKET_ANGLE),
        LEFT_NEAR_HATCH(LEFT_NEAR_HATCH_ROCKET_ANGLE), LEFT_FAR_HATCH(LEFT_FAR_HATCH_ROCKET_ANGLE);

        private double mAngle;

        public double getAngle() {
            return mAngle;
        }

        TargetAngle(double angle) {
            mAngle = angle;
        }
    }

    private ElvTarget mElvTarget;
    private TargetAngle mTargetAngle;
    private Side mSide;

    public Objective(ElvTarget elvTarget)
    {
        mElvTarget = elvTarget;
        mSide = null;
        mTargetAngle = null;
    }

    public Objective(Side side, TargetAngle targetAngle)
    {
        mElvTarget = null;
        mSide = side;
        mTargetAngle = targetAngle;
    }

    public Objective(ElvTarget elvTarget, Side side, TargetAngle targetAngle)
    {
        mElvTarget = elvTarget;
        mSide = side;
        mTargetAngle = targetAngle;
    }


}