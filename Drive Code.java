//package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
	private TalonSRX motorR, motorL;
	private Joystick stickR, stickL;
	private double startTime, time;

	//Initialization
	@Override
	public void robotInit() {
		//Create joysticks
		stickL = new Joystick(0);
		stickR = new joystick(1);

		//Invert L&R motors
		motorL.setInverted(false);
		motorR.setInverted(true);
	}

	@Override
	public void robotPeriodic() {}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	//Calls tank or arcade drive functions
	@Override
	public void teleopPeriodic() {
		TankDrive();
		//ArcadeDrive();
	}

	@Override
	public void testPeriodic() {}

	//Directly set joystick Y axes to motors
	public void TankDrive() {
		motorL.set(ControlMode.PercentOutput, -stickL.getY());
		motorR.set(ControlMode.PercentOutput, -stickR.getY());
	}

	//Y-axis is straight power, X-axis is turn power
	public void ArcadeDrive() {
		motorL.set(ControlMode.PercentOutput, -stickL.getY() + stickR.getX()); //Left = striaght + turn
		motorR.set(ControlMode.PercentOutput, -stickR.getY() - stickL.getX()); //Right = striaght - turn
	}
}