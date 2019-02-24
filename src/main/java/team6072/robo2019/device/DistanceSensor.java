
package team6072.robo2019.device;


import java.io.ByteArrayInputStream;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import team6072.robo2019.logging.*;

/**
 * Use a MaxBotix 1013 untrasonic sensor for range detection.
 * 
 * IMPORTANT: Make sure console out is disabled in the RoboRio configuration
 * http://wpilib.screenstepslive.com/s/currentCS/m/cs_hardware/l/262266-roborio-web-dashboard
 * 
 * Data:
 * 
 * Datasheet: https://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf
 * 
 * Pin 5-Serial Output: By default, the serial output is RS232 format (0 to Vcc)
 * with a 1-mm resolution. If TTL output is desired, solder the TTL jumper pads
 * on the back side of the PCB as shown in the photo to the right. For volume
 * orders, the TTL option is available as no-cost factory installed jumper. The
 * output is an ASCII capital "R", followed by four ASCII character digits
 * representing the range in millimeters, followed by a carriage return (ASCII
 * 13). The maximum distance reported is 5000. The serial output is the most
 * accurate of the range outputs. Serial data sent is 9600 baud, 8 data bits, no
 * parity one stop bit.
 * 
 * Sensor Minimum Distance
 * 
 * The sensor minimum reported distance is 30-cm (11.8 inches). However, the
 * HRLV-MaxSonar-EZ will range and report targets to within 1-mm of the front
 * sensor face. Large targets closer than 30-cm will typically range as 300-mm.
 * 
 * Sensor Operation from 30-cm to 50-cm
 * 
 * Because of acoustic phase effects in the near field, objects between 30-cm
 * and 50-cm may experience acoustic phase cancellation of the returning
 * waveform resulting in inaccuracies of up to 5-mm. These effects become less
 * prevalent as the target distance increases, and has not been observed past
 * 50-cm. For this reason, industrial users that require the highest sensor
 * accuracy are encouraged to mount the HRLV-MaxSonar-EZ from objects that are
 * farther than 50-cm.
 * 
 * On Board - Internal Temperature Compensation
 * 
 * The speed of sound in air increases about 0.6 meters per second, per degree
 * centigrade. Because of this, each HRLV-MaxSonar-EZ is equipped with an
 * internal temperature sensor which allows the sensor to apply a compensation
 * for speed of sound changes.
 * 
 * Filtered Operation - Free-Run The HRLV-MaxSonar-EZ uses an internal 2Hz
 * bandwidth filter to process range data; which reports the latest range every
 * 100mS or 10Hz. This improves the sensor’s performance for accuracy, noise
 * rejection, and reading to reading stability. The filtering in the free-run
 * operation also permits additional acoustic and electrical noise tolerance.
 * 
 */
public class DistanceSensor {

    private static final LogWrapper mLog = new LogWrapper(DistanceSensor.class.getName());

    private static PeriodicLogger mLogPeriodic = new PeriodicLogger(mLog, 10);

    private static DistanceSensor mInstance;

    Notifier mReader;

    public static DistanceSensor getInstance() {
        if (mInstance == null) {
            mInstance = new DistanceSensor();
        }
        return mInstance;
    }


    private SerialPort mPort;


    private DistanceSensor() {
        try {
            mPort = new SerialPort(9600, SerialPort.Port.kOnboard, 8, SerialPort.Parity.kNone,
                    SerialPort.StopBits.kOne);
            // buffer 10 seconds of data - 6 bytes per reading, 10 reading per second
            mPort.setReadBufferSize(6 * 10 * 10);
            mPort.enableTermination(); // readString will only read to \n
            mPort.reset();
            mReader = new Notifier(this::readPort);

        } catch (Exception ex) {
            mLog.severe("DistanceSensor.ctor ex:", ex.getMessage());
            throw ex;
        }
    }
    
    public void resetPort() {
        mPort.reset();
    }

    private final static char[] hexArray = "0123456789ABCDEF".toCharArray();

    public static String bytesToHex(byte[] bytes) {
        char[] hexChars = new char[bytes.length * 2];
        for (int j = 0; j < bytes.length; j++) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = hexArray[v >>> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }
        return new String(hexChars);
    }

    /**
     * This is called by the m_reader Notifier on a periodic basis to read the serial port
     * There are 6 bytes per reading:
     *      R 1234 \n
     */
    private void readPort() {
        try {
            int numBytes = 0;
            try {
                numBytes = mPort.getBytesReceived();
            } catch (Exception ex) {
                mLog.severe("readPort ex in getBytesRx: %s \nmsg: %s \nstack: %s", ex.getClass().getCanonicalName(),
                        ex.getMessage(), ex.getStackTrace());
                mPort.reset();
            }
            mLog.debug("readPort: num bytes %d", numBytes);
            byte[] readData = new byte[numBytes];
            try {
                readData = mPort.read(numBytes);
            }
            catch (Exception ex) {
                mLog.severe("readPort ex in read(numBytes): %s \nmsg: %s \nstack: %s", 
                        ex.getClass().getCanonicalName(), ex.getMessage(), ex.getStackTrace());
                mPort.reset();
            }
            mLog.debug("readPort: %s", bytesToHex(readData));
            // while (mPort.getBytesReceived() > 6) {
            //     String val = mPort.readString();
            //     mLogPeriodic.debug("DistSensor(val):  %s", val);
            //     String num = val.substring(1, 4);
            //     int rangeInMM = Integer.parseInt(num);
            //     double rangeInInches = rangeInMM / 25.4;
            //     mLogPeriodic.debug("DistSensor(inches):  %.3f", rangeInInches);
            // }
        } catch (Exception ex) {
            mLog.severe("readPort ex: %s \nmsg: %s \nstack: %s", ex.getClass().getCanonicalName(),
                    ex.getMessage(), ex.getStackTrace());
            mPort.reset();
        }
    }
    
    public void disable() {
        mReader.stop();
    }

    public void enable() {
        mPort.reset();
        mReader.startPeriodic(1); // read every 50 milliSecs, which should double the rate reading received
    }


}
