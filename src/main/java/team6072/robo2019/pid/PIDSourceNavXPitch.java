package team6072.robo2019.pid;



import team6072.robo2019.subsystems.NavXSys;


public class PIDSourceNavXPitch implements IPIDSource {

    private NavXSys mNavXSys;

    /**
     * pass it an instance of the navXsystem
     * 
     * @param navXSys
     */
    public PIDSourceNavXPitch() {
        mNavXSys = NavXSys.getInstance();
    }

    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    public double pidGet() {
        return mNavXSys.getPitch();
    }

    public void setPIDSourceType(PIDSourceType s) {
    }

}