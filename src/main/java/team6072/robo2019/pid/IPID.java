package team6072.robo2019.pid;


@SuppressWarnings("SummaryJavadoc")
public interface IPID {


    @SuppressWarnings("ParameterName")
    void setPID(double p, double i, double d);

    double getP();

    double getI();

    double getD();

    void setSetpoint(double setpoint);

    double getSetpoint();

    double getError();

    void reset();

    
}