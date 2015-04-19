#include "WPILib.h"
#include "Timer.h"
#include <time.h>
#include "Math.h"

//#include "PowerDistributionPanel.h"
//#include "WPILib.*"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public SampleRobot
{
    // Channels for the wheels
    const static int frontLeftChannel	 = 1;
    const static int rearLeftChannel	 = 0;
    const static int frontRightChannel	 = 3;
    const static int rearRightChannel	 = 2;
    const static int joystick1Channel	 = 0;
    const static int joystick2Channel	 = 1;
    const static int liftMotorChannel    = 5;
    const static int toteChannel         = 4;
    const static int garbageChannel      = 6;

    //Robot
    RobotDrive robotDrive;	// robot drive system

    //Talons
    Talon liftTalon;            //talon on PWM 5
    Talon ToteTalon;            //talon on PWM 4
    Talon garbageTalon;         //talon on PWM 6

	//Joystick
	Joystick stick1;			// driver joystick
	Joystick stick2;			//encoder joystick using buttons 2,3,6,7,9,10,11
	Joystick stick3;


	//Solenoids and Compressor
	Solenoid toteSolenoid;            //CAN 0 Port 1
	Solenoid trashSolenoid;           //CAN 0 Port 0
	Solenoid preTrashSolenoid;        //CAN 0 Port 2
	bool toteToggle = true;
	bool trashToggle = true;

	//Encoder
	Encoder liftEncoder;

	//Ultrasound
	Ultrasonic wallDistance;

	//Limit Switches
	DigitalInput minHeight;

	//Lights
	DigitalOutput sonarLight;

	//From PDP
	double voltage;		    //input voltage to the PDP
	double current;			//current draw on the PDP
	bool ResetEncoder = false;
	float matchTime;
	int offset = -60;
public:
	Robot ():
			robotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel),           //robot drive talons
			liftTalon(liftMotorChannel), ToteTalon(toteChannel), garbageTalon(garbageChannel),//Other talons
			stick1(joystick1Channel),stick2(joystick2Channel), stick3(3), toteSolenoid(0,1), trashSolenoid(0,0), preTrashSolenoid(0,2),    //sticks and solenoids
			liftEncoder(8,9,true, Encoder::k4X),wallDistance(3,4), minHeight(6), sonarLight(5)   //encoders and limit switches

		//initiated in the same order as they are declared above
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);	// you may need to change or remove this to match your robot
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
		DriverStation::ReportError("Camera Operation");
		wallDistance.SetAutomaticMode(true);
		//double distance = 0; //use diameter and M_PI
	}//runs in mechanum
	//-----------------------------------------------------------------------------------------------------------------------------------------------------
	//AutonomousAutonomousAutonomousAutonomousAutonomousAutonomousAutonomousAutonomousAutonomousAutonomousAutonomousAutonomousAutonomousAutonomousAutonomous
	//-----------------------------------------------------------------------------------------------------------------------------------------------------
	void Autonomous(void)
	{
		//time_t start = time(0);
		float startTime = Timer::GetFPGATimestamp();
		preTrashSolenoid.Set(true);
		while(RobotBase::IsAutonomous())
		{
			matchTime = (Timer::GetFPGATimestamp() -startTime);
			DriverStation::ReportError(std::to_string(matchTime) + "\n");
			if(minHeight.Get())
			{
				liftTalon.Set(0);
				liftEncoder.Reset();
			}
			//RMV is Inverted
			//Clockwise = (1,1)
			//Forward = (1,-1)
			//Backward = (-1,1)
			//CounterClockWise = (-1,-1)
			if(DigitalInput(0).Get() == 0)//DIO thing is turned to 0)//drive forward and stop
			{
				DriverStation::ReportError("CCW: ");
				toteSolenoid.Set(false);
				if(-liftEncoder.Get()*18/25 < 300 && matchTime > 3.5 && matchTime < 8)
				{
					liftTalon.Set(1);
				}
				else
					liftTalon.Set(0);
				if(matchTime < 4)//GRAB TRASH
					trashSolenoid.Set(true);
				else if(matchTime < 5.5)//Run Backwards
					robotDrive.SetLeftRightMotorOutputs(-0.55, 0.4);
				else if(matchTime < 6.8)//Stop Driving
					robotDrive.SetLeftRightMotorOutputs(-0.3, -0.3);//COUNTER CLOCKWISE
				else
				{
					toteSolenoid.Set(false);
					robotDrive.SetLeftRightMotorOutputs(0.0, 0.0);
					if(-liftEncoder.Get()*18/25 > 0 && !minHeight.Get())
						liftTalon.Set (-1);
					else
						liftTalon.Set (0);
				}
			}
			else if(DigitalInput(1).Get() == 0)//DIO thing turned to 1)//DRIVE WITH A TRASH CAN AND TURN CCW
			{
				toteSolenoid.Set(false);
				DriverStation::ReportError("CW: ");
				if(-liftEncoder.Get()*18/25 < 300 && matchTime > 3.5 && matchTime < 8)
				{
					liftTalon.Set(1);
				}
				else
					liftTalon.Set(0);
				if(matchTime < 4)//GRAB TRASH
					trashSolenoid.Set(true);
				else if(matchTime < 5.5)//Run Backwards
					robotDrive.SetLeftRightMotorOutputs(-0.55, 0.4);
				else if(matchTime < 7.1)//Stop Driving
					robotDrive.SetLeftRightMotorOutputs(0.3, 0.3);//CLOCKWISE
				else
				{
					toteSolenoid.Set(false);
					robotDrive.SetLeftRightMotorOutputs(0.0, 0.0);
					if(-liftEncoder.Get()*18/25 > 0 && !minHeight.Get())
						liftTalon.Set (-1);
					else
						liftTalon.Set (0);
				}
			}
			else
			{
				DriverStation::ReportError("NOTHING: ");
			}
		}
	}
	//-----------------------------------------------------------------------------------------------------------------------------------------------------
	//TeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleopTeleop
	//-----------------------------------------------------------------------------------------------------------------------------------------------------
	void OperatorControl()
	{
		//IMAQdxStartAcquisition(session);
		robotDrive.SetSafetyEnabled(false);
		SmartDashboard::init();

		double baseMotorValue = 0.0;
		bool liftTote         =false;
		bool lowerTote        =false;
		bool toteButtonPushed =false;
		bool releaseButton    =false;
		int releaseTotes      = 0;
		int numberOfTotes     = 0; //accounts for totes and trash cans
		int currentEncoderVal = (-liftEncoder.Get())*(3.666); // 18/25 * 26.9/71     .2728    3.666
		int distanceToMove    = 0; //Current Target for the Encoder motor to reach
		int rangeToWall       = 0;

		preTrashSolenoid.Set(true);

		while (IsOperatorControl() && IsEnabled())
		{
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
			//-----------------------------------------------------------------------------------------------------------
			//MechanumDrive
			if(stick1.GetRawButton(1))
				robotDrive.MecanumDrive_Cartesian(stick1.GetX(), stick1.GetY(), stick1.GetZ()*.5);
			else
				robotDrive.MecanumDrive_Cartesian(stick1.GetX()*.5, stick1.GetY()*.5, stick1.GetZ()*.2);

			//-----------------------------------------------------------------------------------------------------------
			//Buttons that Control the Solenoids for trash and arms
			toteToggle = !stick2.GetRawButton(1);

			//trash Toggle
			trashToggle = !stick2.GetRawButton(2);

			///Encoder Distance
			currentEncoderVal = (-liftEncoder.Get())*(3.666);

			//Ultrasonic Distance To Wall
			rangeToWall = wallDistance.GetRangeInches();

			//-----------------------------------------------------------------------------------------------------------
			//Buttons that Control the Arms Pulling or Pushing
			if(stick2.GetRawButton(3))
			{
				ToteTalon.Set(1);//totes sucking in (I guess)
			}
			else if(stick2.GetRawButton(4))
			{
				ToteTalon.Set(-1);//totes sucking in (I guess)
			}
			else
			{
				ToteTalon.Set(0);//totes sucking in (I guess)
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
			else if(stick2.GetRawButton(8) && !lowerTote && !liftTote && !releaseTotes && numberOfTotes < 5)//Move Upward a Step
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
						distanceToMove = 1310 + offset;
						break;
					case 2:
						distanceToMove = 3350 + offset;
						break;
					case 3:
						distanceToMove = 5450 + offset;
						break;
					case 4:
						distanceToMove = 7500 + offset;
						break;
					case 5:
						distanceToMove = 9580 + offset; //change to 1.5 inches
						break;
					case 6:
						distanceToMove = 10250 + offset; //change to 1.5 inches
						break;
				}
			}

			//-----------------------------------------------------------------------------------------------------------
			//Code to run the liftTalon to the position of the next Tote spot
			if (stick2.GetRawButton(7) || stick2.GetRawButton(6))
			{
				liftTote = false;
				lowerTote = false;
				//releaseTotes = 0;
			}
			if (releaseTotes == 1 && releaseButton)
			{
				liftTalon.Set(-.5);
				toteToggle = false;
				if(currentEncoderVal < 8700) //change to trashcan Height
				{
					trashToggle = false;
					releaseButton = false;
				}
			}
			else if(releaseTotes == 2 && releaseButton)
			{
				liftTalon.Set(-.5);
				toteToggle = false;
				trashToggle = false;
				if(minHeight.Get())
				{
					releaseButton = false;
					releaseTotes = 0;
					liftTalon.Set(0);
				}
			}
			else
			{
				if(liftTote && (currentEncoderVal < distanceToMove))//1060 is the distance of one tote movement
				{
					liftTalon.Set(1);
					toteToggle = false;
					lowerTote = false;
				}
				else if(!minHeight.Get() && lowerTote && currentEncoderVal > distanceToMove )//1060 is the distance of one tote movement
				{
					liftTalon.Set(-1);
					toteToggle = false;
					liftTote = false;
				}
				else
				{
					liftTote = false;
					lowerTote = false;

					//manual control of the liftMotor
					if(!minHeight.Get())
					{
						if(stick2.GetRawButton(7)) //motor runs downwards
						{
							if(currentEncoderVal > 1000)
								liftTalon.Set(-.5);
							else
								liftTalon.Set(-.3);
						}
						else if(stick2.GetRawButton(6)) //motor runs upwards
						{
							if(currentEncoderVal > 9500)
								liftTalon.Set(.5);
							else
								liftTalon.Set(.7);
						}
						else
							liftTalon.Set(0);
					}
					else if (stick2.GetRawButton(7))
					{
						if(currentEncoderVal > 1000)
							liftTalon.Set(-.5);
						else
							liftTalon.Set(-.3);
					}
					else if (minHeight.Get() && stick2.GetRawButton(6))
						liftTalon.Set(.5);
					else
						liftTalon.Set(0);
				}

			}
			if(numberOfTotes > 1)
			{
				baseMotorValue = .06 * numberOfTotes;
			}
			else
			{
				baseMotorValue = 0.0;
			}
			if(liftTalon.Get() <=0.05 && liftTalon.Get() >= -.05)
				liftTalon.Set(baseMotorValue);
			if(stick3.GetY() > .05 || stick3.GetY() < -.05)
			{
				if (stick3.GetY() > 0)
					liftTalon.Set(stick3.GetY() + baseMotorValue);
				else
					liftTalon.Set(stick3.GetY());
			}
			//-----------------------------------------------------------------------------------------------------------
			//TrashTalon Code
			if(currentEncoderVal < 500)//Change from 1000
			{
				garbageTalon.Set(1);
			}
			else
			{
				garbageTalon.Set(-.6);
			}

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
			//Set the solenoids only once

			toteSolenoid.Set(toteToggle);
			trashSolenoid.Set(trashToggle);

			//-----------------------------------------------------------------------------------------------------------
			DriverStation::ReportError(std::to_string(rangeToWall) + "\n");
			//SmartDashboardThings

			SmartDashboard::PutNumber("currentEV", (currentEncoderVal));
			SmartDashboard::PutBoolean ("Lift Tote", liftTote); ///remove
			SmartDashboard::PutBoolean ("Lower Tote", lowerTote); //remove?
			SmartDashboard::PutBoolean ("switchMinHeight", minHeight.Get()); //remove
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
