import QuickPID.PID;

public class Main {
	public static void main() {
		double outputValue = 0.0;
		double sensorValue = 0.0;
		double targetValue = 0.0;
		double kP=0.0;
		double kI=0.0;
		double kD=0.0;
		
		PID p = new PID(kP, kI, kD);
		p.Reset();
		
		p.SetSetpoint(targetValue);
		
		while (sensorValue < targetValue)
		{		
			//read new sensor value;
			outputValue = p.GetOutput(sensorValue);
		}
	}
}
