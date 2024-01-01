package frc2023.util.input;

public class Joystick extends edu.wpi.first.wpilibj.Joystick {

	public Joystick(int port) {
		super(port);
	}

	public String getName() {
		return "Joystick " + getPort();
	}
}
