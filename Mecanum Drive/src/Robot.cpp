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
	DigitalInput minHeight;
	DigitalInput toteInPlace;
	//From PDP
	double voltage;		    //input voltage to the PDP
	double current;			//current draw on the PDP
public:
	Robot ():
			robotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel),           //robot drive talons
			liftTalon(liftMotorChannel), rightToteTalon(rightToteChannel), leftToteTalon(leftToteChannel),//Other talons
			stick1(joystick1Channel),stick2(joystick2Channel), toteSolenoid(0,1), trashSolenoid(0,0),     //sticks and solenoids
			liftEncoder(8,9,true, Encoder::k4X), maxHeight(7), minHeight(6), toteInPlace(5)             //encoders and limit switches

		//initiated in the same order as they are declared above
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);	// you may need to change or remove this to match your robot
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
		DriverStation::ReportError("Camera works haha");
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
		double baseMotorValue = 0.0;
		bool liftTote = false;
		bool lowerTote = false;
		bool toteButtonPushed =  false;
		bool releaseButton = false;
		int releaseTotes = 0;
		int numberOfTotes = 0; //accounts for totes and trash cans
		int currentEncoderVal = (-liftEncoder.Get())*18/25;
		int distanceToMove = 0;
		//CameraServer::GetInstance()->StartAutomaticCapture("cam0");
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

			//-----------------------------------------------------------------------------------------------------------
			//Buttons that Control the Solenoids for trash and arms
			toteToggle = !stick2.GetRawButton(1);
			toteSolenoid.Set(toteToggle);
			//trash Toggle
			trashToggle = !stick2.GetRawButton(2);
			trashSolenoid.Set(trashToggle);

			///Encoder Distance
			currentEncoderVal = (-liftEncoder.Get())*18/25;

			//-----------------------------------------------------------------------------------------------------------
			//Buttons that Control the Arms Pulling or Pushing
			if(stick2.GetRawButton(3))
			{
				rightToteTalon.Set(1);//totes sucking in (I guess)
				leftToteTalon.Set(-1);
			}
			else if(stick2.GetRawButton(4))
			{
				rightToteTalon.Set(-1);//totes sucking in (I guess)
				leftToteTalon.Set(1);
			}
			else
			{
				rightToteTalon.Set(0);//totes sucking in (I guess)
				leftToteTalon.Set(0);
			}

			//-----------------------------------------------------------------------------------------------------------
			//Buttons that Control the Lift Motor
			if(stick2.GetRawButton(5) && !releaseButton)//lower to 4000 and drop the trash can at the end
			{
				numberOfTotes = 0;
				releaseButton = true;
				if(releaseTotes == 0)//lower the tote below
					releaseTotes = 1;
				else if (releaseTotes == 1)//lower till it stops
					releaseTotes = 2;
			}
			else if(stick2.GetRawButton(8) && !lowerTote && !liftTote && !releaseTotes && numberOfTotes < 5 && !maxHeight.Get())//Move Upward a Step
			{
				liftTote = true;
				numberOfTotes++;
				toteButtonPushed = true;

			}
			else if(stick2.GetRawButton(9) && !liftTote && !lowerTote && !releaseTotes && numberOfTotes > 0 && !minHeight.Get())//Move Downward a Step
			{
				lowerTote = true;
				numberOfTotes--;
				toteButtonPushed = true;
			}
			else //Nothing Happens
			{
				toteButtonPushed = false;
			}

			if(toteButtonPushed)
			{
				switch(numberOfTotes)
				{
					case 0:
						distanceToMove = 0;
						break;
					case 1:
						distanceToMove = 1100;
						break;
					case 2:
						distanceToMove = 2190;
						break;
					case 3:
						distanceToMove = 3280;
						break;
					case 4:
						distanceToMove = 4360;
						break;
					case 5:
						distanceToMove = 4650; //change to 1.5 inches
						break;
					default:
						distanceToMove = 1060*numberOfTotes;
						break;
				}
			}

			//-----------------------------------------------------------------------------------------------------------
			//Code to run the liftTalon to the position of the next Tote spot
			if (releaseTotes == 1 && releaseButton)
			{
				liftTalon.Set(-1);
				toteSolenoid.Set(false);
				if(currentEncoderVal < 3900) //change to trashcan Height
				{
					trashSolenoid.Set(false);
					releaseButton = false;
				}
			}
			else if(releaseTotes == 2 && releaseButton)
			{
				liftTalon.Set(-1);
				toteSolenoid.Set(false);
				trashSolenoid.Set(false);
				if(minHeight.Get())
				{
					releaseButton = false;
					releaseTotes = 0;
					liftTalon.Set(0);
				}
			}
			else
			{
				if(!maxHeight.Get() && liftTote && (currentEncoderVal < distanceToMove) )//1060 is the distance of one tote movement, maxheight limits the motion of the lift motor.
				{
					liftTalon.Set(1);
					toteSolenoid.Set(false);
					lowerTote = false;
				}
				else if(!minHeight.Get() && lowerTote && currentEncoderVal > distanceToMove )//1060 is the distance of one tote movement
				{
					liftTalon.Set(-1);
					toteSolenoid.Set(false);
					liftTote = false;
				}
				else
				{
					liftTote = false;
					lowerTote = false;

					//manual control of the liftMotor
					if(!maxHeight.Get() && !minHeight.Get()) //maxHeight is not tiggered move whatever
					{
						if(stick2.GetRawButton(7)) //motor runs downwards
							liftTalon.Set(-1);
						else if(stick2.GetRawButton(6)) //motor runs upwards
							liftTalon.Set(1);
						else
							liftTalon.Set(0);
					}
					else if (maxHeight.Get() && stick2.GetRawButton(7)) //maxHeight is triggered so the motor can only run downwards
						liftTalon.Set(-1);
					else if (minHeight.Get() && stick2.GetRawButton(6))
						liftTalon.Set(1);
					else
						liftTalon.Set(0);
				}

			}
			if(numberOfTotes > 1)
			{
				baseMotorValue = .03 * numberOfTotes;
			}
			else
			{
				baseMotorValue = 0.0;
			}
			if(liftTalon.Get() <=0.05 && liftTalon.Get() >= -.05 && !maxHeight.Get())
				liftTalon.Set(baseMotorValue);

			//-----------------------------------------------------------------------------------------------------------
			//Limit Switches
			if(minHeight.Get())//get a known location for the bottom solenoid
			{
				liftEncoder.Reset();
				numberOfTotes = 0;
			}

			//-----------------------------------------------------------------------------------------------------------
			//DriveJoystickLEDs

			for(int ii = 0; ii <= numberOfTotes + 1; ii++)
				stick2.SetOutput(ii,true);

			for(int kk = numberOfTotes+1; kk <= 6; kk++)
				stick2.SetOutput(kk,false);

			//-----------------------------------------------------------------------------------------------------------
			//SmartDashboardThings
			SmartDashboard::PutNumber("currentEV", (currentEncoderVal));
			SmartDashboard::PutBoolean ("Lift Tote", liftTote); ///remove
			SmartDashboard::PutBoolean ("Lower Tote", lowerTote); //remove?
			SmartDashboard::PutBoolean ("switchMinHeight", minHeight.Get()); //remove
			SmartDashboard::PutBoolean ("switchMaxHeight", maxHeight.Get()); //remove
			SmartDashboard::PutNumber("Encoder stuff", (-liftEncoder.Get())*18/25); // 18/25 = 360/500 this converts the encoder count to degrees (360 degrees = 500 count)
			SmartDashboard::PutNumber("Encoder graph", (-liftEncoder.Get())*18/25); // 18/25 = 360/500 this converts the encoder count to degrees (360 degrees = 500 count)
			SmartDashboard::PutNumber("Lift Talon", liftTalon.Get());
			SmartDashboard::PutNumber("NumberOfTotes", (numberOfTotes));
			SmartDashboard::PutNumber("DistanceToMoveTo", (distanceToMove));

			Wait(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}
};

START_ROBOT_CLASS(Robot);
