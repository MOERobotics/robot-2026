package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

import java.util.Arrays;
import java.util.concurrent.ConcurrentLinkedQueue;

public class SerialReader extends Thread{
    SerialPort serialPort = new SerialPort(115200, SerialPort.Port.kMXP);
    public final ConcurrentLinkedQueue<String> incomingData = new ConcurrentLinkedQueue<>();

    public SerialReader(){
        serialPort.enableTermination();
        serialPort.setReadBufferSize(1024);

    }



    @Override
    public void run() {
        while (true) {
            String data = serialPort.readString();
            incomingData.add(data);
        }


    }


}
