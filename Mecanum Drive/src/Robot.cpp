#include "WPILib.h"
//#include "PowerDistributionPanel.h"
//#include "WPILib.*"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{
    // Channels for the wheels
    const static int frontLeftChannel	= 1;
    const static int rearLeftChannel	= 0;
    const static int frontRightChannel	= 3;
    const static int rearRightChannel	= 2;
    //Joystick
    const static int joystickChannel	= 0;
    Joystick stick;			// only joystick
    //Drive
	RobotDrive robotDrive;	// robot drive system
	//Solenoids and Compressor
	Solenoid toteSolenoid;// CAN 0 Port 1
	Solenoid trashSolenoid; //CAN 0 Port 0
	bool toteToggle = false;
	bool trashToggle = false;
	//From PDP
	double voltage;		    //input voltage to the PDP
	double current;			//current draw on the PDP

public:
	Robot() :
			robotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel),	stick(joystickChannel), toteSolenoid(0,1), trashSolenoid(0,0)
		//initiated in the same order as they are declared above
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);	// you may need to change or remove this to match your robot
	}//runs in mechanum
	void OperatorControl()
	{
		robotDrive.SetSafetyEnabled(false);
		while (IsOperatorControl() && IsEnabled())
		{
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
			robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(), stick.GetZ());
			//tote Toggle
			toteToggle = !stick.GetRawButton(7);
			toteSolenoid.Set(toteToggle);
			//trash Toggle
			trashToggle = !stick.GetRawButton(8);
			trashSolenoid.Set(trashToggle);

			Wait(0.005); // wait 5ms to avoid hogging CPU cycles

		}
	}
};

START_ROBOT_CLASS(Robot);
