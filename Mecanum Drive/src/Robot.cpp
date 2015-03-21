#include "WPILib.h"
#include "Math.h"
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
    const static int joystick1Channel	= 0;
    const static int joystick2Channel	= 1;
    const static int liftMotorChannel   = 5;
    const static int rightToteChannel   = 4;
    const static int leftToteChannel    = 6;

    //Robot
    RobotDrive robotDrive;	// robot drive system

    //Talons
    Talon liftTalon; //talon on PWM 5
    Talon rightToteTalon;//talon on PWM 4
    Talon leftToteTalon;//talon on PWM 6
    //Talon rightToteTalon; //talon on PWM 4
    //Talon leftToteTalon; //talon on PWM 6

	//Joystick
	Joystick stick1;			// driver joystick
	Joystick stick2;			//encoder joystick using buttons 2,3,6,7,9,10,11


	//Solenoids and Compressor
	Solenoid toteSolenoid;            // CAN 0 Port 1
	Solenoid trashSolenoid;           //CAN 0 Port 0
	bool toteToggle = true;
	bool trashToggle = true;

	//Encoder
	Encoder liftEncoder;

	//Limit Switches
	DigitalInput maxHeight;
	DigitalInput bottomReset;
	DigitalInput toteInPlace;
	//From PDP
	double voltage;		    //input voltage to the PDP
	double current;			//current draw on the PDP
public:
	Robot ():
			robotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel),           //robot drive talons
			liftTalon(liftMotorChannel), rightToteTalon(rightToteChannel), leftToteTalon(leftToteChannel),//Other talons
			stick1(joystick1Channel),stick2(joystick2Channel), toteSolenoid(0,1), trashSolenoid(0,0),     //sticks and solenoids
			liftEncoder(8,9,true, Encoder::k4X), maxHeight(7), bottomReset(6), toteInPlace(5)             //encoders and limit switches

		//initiated in the same order as they are declared above
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);	// you may need to change or remove this to match your robot
		//Camera Stuff
		/*frame = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
				//the camera name (ex "cam0") can be found through the roborio web interface
				imaqError = IMAQdxOpenCamera("cam0", IMAQdxCameraControlModeController, &session);
				if(imaqError != IMAQdxErrorSuccess) {
					DriverStation::ReportError("IMAQdxOpenCamera error: " + std::to_string((long)imaqError) + "\n");
				}
				imaqError = IMAQdxConfigureGrab(session);
				if(imaqError != IMAQdxErrorSuccess) {
					DriverStation::ReportError("IMAQdxConfigureGrab error: " + std::to_string((long)imaqError) + "\n");
				}*/
		//End Camera Stuff
		while(false)
		{
			//run a motor until the limit switch triggers
			//liftEncoder.reset();
		}
		liftEncoder.Reset();//remove when the above lines work
		//liftEncdoer.Start(); //not necessary
		//int encoderCount = liftEncoder.Get();
		//double distance = 0; //use diameter and M_PI
	}//runs in mechanum
	void OperatorControl()
	{
		//IMAQdxStartAcquisition(session);
		robotDrive.SetSafetyEnabled(false);
		SmartDashboard::init();
		liftEncoder.Reset();
		bool liftTote = false;
		bool lowerTote = false;
		int oldEncoderVal = liftEncoder.Get()*18/25;
		int numberOfTotes = 0; //accounts for totes and trash cans
		int currentEncoderVal = liftEncoder.Get()*18/25;
		int distanceToMove = 1060;
		while (IsOperatorControl() && IsEnabled())
		{
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
			robotDrive.MecanumDrive_Cartesian(stick1.GetX(), stick1.GetY(), stick1.GetZ()*.5);
			/*
			//Camera Stuff
			// acquire images
			//IMAQdxStartAcquisition(session);
			// grab an image, draw the circle, and provide it for the camera server which will
			// in turn send it to the dashboard.
				IMAQdxGrab(session, frame, true, NULL);
				if(imaqError != IMAQdxErrorSuccess) {
					DriverStation::ReportError("IMAQdxGrab error: " + std::to_string((long)imaqError) + "\n");
				}
				else {
					imaqDrawShapeOnImage(frame, frame, { 10, 10, 100, 100 }, DrawMode::IMAQ_DRAW_VALUE, ShapeMode::IMAQ_SHAPE_OVAL, 0.0f);
				CameraServer::GetInstance()->SetImage(frame);
				}
				Wait(0.005);				// wait for a motor update time
			}
			// stop image acquisition
			IMAQdxStopAcquisition(session);
			*/

			//tote Toggle
			toteToggle = !stick2.GetRawButton(3);
			toteSolenoid.Set(toteToggle);
			//trash Toggle
			trashToggle = !stick2.GetRawButton(2);
			trashSolenoid.Set(trashToggle);

			//Drive the tote pull in talons
			if(stick2.GetRawButton(7))
			{
				rightToteTalon.Set(1);//totes sucking in (I guess)
				leftToteTalon.Set(-1);
			}
			if(stick2.GetRawButton(6))
			{
				rightToteTalon.Set(-1);//totes sucking in (I guess)
				leftToteTalon.Set(1);
			}

			//Automatically move one crate height
			currentEncoderVal = liftEncoder.Get()*18/25;
			if(stick2.GetRawButton(9))
			{
				liftTote = false;
				liftEncoder.Reset();
			}
			if(stick2.GetRawButton(11) && !lowerTote && !liftTote && numberOfTotes < 6)
			{
				liftTote = true;
				numberOfTotes++;
			}
			if(stick2.GetRawButton(10) && !liftTote && !lowerTote && numberOfTotes > 0)
			{
				lowerTote = true;
				numberOfTotes--;
			}

			//Driving the tote lifter and acknowledging limits
			switch(numberOfTotes)
			{
				case 0:
					distanceToMove = 10; //change to trashcanheight
					break;
				case 5:
					distanceToMove = 10; //change to 1.5 inches
					break;
				default:
					distanceToMove = 1060;
					break;
			}

			if(!maxHeight.Get() && liftTote && (abs(currentEncoderVal- oldEncoderVal)< distanceToMove) )//1060 is the distance of one tote movement, maxheight limits the motion of the lift motor.
			{
				liftTalon.Set(1);
			}
			else if(!maxHeight.Get() && lowerTote && (abs(currentEncoderVal- oldEncoderVal)< distanceToMove) )//1060 is the distance of one tote movement
			{
				liftTalon.Set(-1);
				numberOfTotes--;
			}
			else
			{
				oldEncoderVal = liftEncoder.Get()*18/25;
				lowerTote = false;
				liftTote = false;
				liftTalon.Set(0); // accounts for switch being pressed
				//manual control of the liftMotor
				if(!maxHeight.Get()) //maxHeight is not tiggered move whatever
					liftTalon.Set(-stick2.GetY());
				else if (-stick2.GetY() < 0) //maxHeight is triggered so the motor can only run downwards
					liftTalon.Set(-stick2.GetY());
			}

			//Limit switch properties
			if(!bottomReset.Get())//get a known location for the bottom solenoid
			{
				liftEncoder.Reset();
			}

			/*if(          toteInPlace.Get())
			{
				liftTote = true;
			}*/
			//SmartDashboardThings
			SmartDashboard::PutNumber("Encoder stuff", (-liftEncoder.Get())*18/25); //remove or convert to percent/progress
			SmartDashboard::PutNumber("currentEV", (currentEncoderVal));
			SmartDashboard::PutNumber("oldEV", (oldEncoderVal));
			SmartDashboard::PutNumber("subtractedEV", (currentEncoderVal-oldEncoderVal));
			SmartDashboard::PutBoolean ("liftTote", liftTote); ///remove
			SmartDashboard::PutBoolean ("switchBR", !bottomReset.Get()); //remove
			SmartDashboard::PutBoolean ("switchMH", maxHeight.Get()); //remove
			SmartDashboard::PutNumber("Encoder stuff", (-liftEncoder.Get())*18/25); // 18/25 = 360/500 this converts the encoder count to degrees (360 degrees = 500 count)
			SmartDashboard::PutNumber("Encoder stuff2", (-liftEncoder.Get())*18/25); // 18/25 = 360/500 this converts the encoder count to degrees (360 degrees = 500 count)
			SmartDashboard::PutNumber("Joystick2 stuff", -stick2.GetY()+1);

			Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}
};

START_ROBOT_CLASS(Robot);
