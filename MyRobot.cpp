#include "WPILib.h"
#include <math.h>
#include "Target.h"
#include "DashboardDataSender.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include <nivision.h>
#include "math.h"


class RobotDemo: public SimpleRobot {
	AxisCamera &camera;
	Joystick leftStick;
	Joystick rightStick;
	Joystick manip;
	Compressor compressor;
	Solenoid shift;
	CANJaguar leftJag1;
	CANJaguar leftJag2;
	CANJaguar rightJag1;
	CANJaguar rightJag2;
	CANJaguar shooterJag1;
	CANJaguar turretRotate;
	//	CANJaguar eyeOfGreen;
	Relay roller1;
	Relay roller2;
	Relay turretRoller;
	Encoder leftSide;
	Encoder rightSide;
	Solenoid bridgeManip;
	float shooterValue;
	float rotaterSetPoint;
	bool HighGear;
	double leftPrevDist, rightPrevDist;
	double leftPrevTime, rightPrevTime;
	bool autoTrackState;
	DashboardDataSender dds;
	DigitalInput intakeSwitch;
	bool isGoingUp;
		Notifier checkInTake;
		Notifier checkJoystick;
	Notifier checkMotorSpeed;
	volatile float setPoint;
	Notifier printData;
	Notifier checkDistanceDriving;
	volatile float distanceDriven;
	Gyro theGyro;
	volatile float gyroAngle;
	volatile float distance;
	bool keepDriving;
	float offsetAim;
	bool suckTime;
	bool driveTime;
	float currentDistance;
	Notifier cameraThread;
	volatile float x1, x2, x3, x4;
	volatile float y1, y2, y3, y4;
	volatile float currentCenter;
	volatile float centerNeeded;
	volatile float setValue;
	volatile float shooterSpeed;
	bool canSetRPM;
	float driveSpeed;
	float correctSpeed;
	bool footLeft;
	bool footRight;
	bool shootTime;
	bool ignoreLoop;
	bool goBackwards;
	bool stopDriving;
	int rpmAdjuster;
	volatile bool suckingUp;
public:
	RobotDemo(void) :
		camera(AxisCamera::GetInstance()),
				leftStick(1),
				rightStick(2),
				manip(3),
				compressor(1, 1),
				shift(1),
				leftJag1(1),
				leftJag2(2),
				rightJag1(3),
				rightJag2(4),
				shooterJag1(7),
				turretRotate(8),
				//eyeOfGreen(9),
				roller1(2, Relay::kBothDirections),
				roller2(4, Relay::kBothDirections),
				turretRoller(3, Relay::kBothDirections),
				leftSide(4, 5, Encoder::k4X),
				rightSide(10, 11, Encoder::k4X),
				bridgeManip(2),
				HighGear(false),
				//autoTrackState(false),
				dds(),
				intakeSwitch(6),
				isGoingUp(false),
					checkInTake(RobotDemo::checkIntakes, this),
					checkJoystick(RobotDemo::checkForButton, this),
				checkMotorSpeed(RobotDemo::checkSpeed, this),
				printData(RobotDemo::printValues, this),
				checkDistanceDriving(RobotDemo::checkDistanceSpeed, this),
				theGyro(2), cameraThread(RobotDemo::updateCameraStuff, this){
		
		rpmAdjuster = 0;
		centerNeeded = 160;
		//setPoint = 2850;
		offsetAim = -4;
		keepDriving = true;
		//	printf("Setting camera parameters\n");
		theGyro.SetSensitivity(-.007);
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(20);
		//camera.WriteBrightness(0);		
		shooterJag1.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
		leftJag1.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
		leftJag2.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
		rightJag1.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
		rightJag2.ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
		//eyeOfGreen.ChangeControlMode(CANJaguar::kCurrent);
		turretRotate.ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
		turretRotate.ChangeControlMode(CANJaguar::kVoltage);
		//turretRotate.SetVoltageRampRate(24);
		turretRotate.ConfigEncoderCodesPerRev(3400);
		//turretRotate.SetPID(1065, .005, .25);
		turretRotate.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
		turretRotate.EnableControl(0.0);
		shooterJag1.SetVoltageRampRate(200); //24 VPS ramp up
		//	shooterJag1.ChangeControlMode(CANJaguar::kSpeed);
		shooterJag1.ConfigEncoderCodesPerRev(305);
		shooterJag1.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
		//	shooterJag1.SetPID(1,0,0);
		//   shooterJag1.EnableControl(0.0);		
		compressor.Start();
		leftSide.SetDistancePerPulse(0.0063612959214038);
		rightSide.SetDistancePerPulse(0.0063612959214038);
		leftSide.Start();
		rightSide.Start();
		checkMotorSpeed.StartPeriodic(.05);
		checkDistanceDriving.StartPeriodic(.05);
		cameraThread.StartPeriodic(.25);
		//printData.StartPeriodic(.1);
		//	turretRotate.ConfigSoftPositionLimits(-.813232,.099258);
		setPoint = 500;
		//myRobot.SetExpiration(0.1);
		suckTime = false;
		driveTime = false;
		driveSpeed = .8;
		correctSpeed = 1;
		footRight = false;
		footLeft = false;
	}

	float giveRPM(float xOne, float xTwo)
	{		
		

		float distanceTable[] =  { 200, 180, 166, 154, 143, 132, 123, 115, 108,102 }; 
		float rpmTable[] = { 2530, 2520, 2520, 2590, 2650, 2760, 2770, 2820,2920, 3020 };
		if(xOne == 0 || xTwo == 0)
		{
			xOne = 5;
			xTwo = 4;
		}
		/*float distanceTable[] = { 200, 180, 166, 154, 143, 132, 123, 115, 108,102 };
		float rpmTable[] = { 2530, 2520, 2520, 2590, 2650, 2760, 2770, 2820,2920, 3020 };
		*/
		//rpm calculation and interpolation
			currentDistance = fabs(xTwo - xOne); //distance between the right target - the left target
														  //the higher number the closer we are
			//printf("CurrentDistance: %f \n", currentDistance);
				//this if block tests the 2 extreme values we could encounter, 200 being closest.
				if (currentDistance > 200) {
					shooterSpeed = 2500;
				} else if (currentDistance < 102) {
					shooterSpeed = 3300;
				}



				//if not at the actual positions, loop through
				float valueToReturn;
				int position;

				for (int i = 0; i <= 9; i++) {
					if ((currentDistance <= distanceTable[i])
							&& (currentDistance >= distanceTable[i + 1])) {
						//assign a "position" to be used later on
						//200,180,166,154,143,132,123,115,108,102
						position = i;

					}
				}
				
				//here is where position is used, for the linear interp.
				valueToReturn = (((rpmTable[position] - rpmTable[position + 1])
						/ (distanceTable[position] - distanceTable[position + 1]))
						* (currentDistance - distanceTable[position + 1]))
						+ rpmTable[position + 1];
				return valueToReturn + rpmAdjuster;
	}
	
	static void checkDistanceSpeed(void *s) {
		//	RobotDemo &self = *reinterpret_cast<RobotDemo*>(s);
		//	self.distanceDriven = self.absd((self.leftSide.GetDistance(), self.rightSide.GetDistance()) /2 );
		/*	if(self.distanceDriven < 10)
		 {
		 self.leftJag1.Set(.3);
		 self.leftJag2.Set(.3);
		 self.rightJag1.Set(-.3);
		 self.rightJag2.Set(-.3);
		 }
		 else
		 {
		 self.leftJag1.Set(0);
		 self.leftJag2.Set(0);
		 self.rightJag1.Set(0);
		 self.rightJag2.Set(0);
		 }
		 */

	}

	static void checkSpeed(void *s) {
		RobotDemo &self = *reinterpret_cast<RobotDemo*> (s);
		if(self.setPoint == 0)
		{
			self.shooterJag1.Set(0);
		}
		else if(self.shooterJag1.GetSpeed() > self.setPoint)
		{
			self.shooterJag1.Set(0);
		}
		else if(self.shooterJag1.GetSpeed() < self.setPoint)
		{
			self.shooterJag1.Set(1);
		}
		/*if (self.setPoint > 3300) {
			self.setPoint = 3300;
		}*/
		/*
		if (self.shooterJag1.GetSpeed() < 500) {
			self.shooterJag1.Set(1);
		} else if ((self.shooterJag1.GetSpeed() - self.setPoint) > 500) {
			self.shooterJag1.Set(0);
		} else if (self.shooterJag1.GetSpeed() > self.setPoint) {
			self.shooterJag1.Set(.1);
		} else if ((self.setPoint - self.shooterJag1.GetSpeed()) > 100) {
			self.shooterJag1.Set(1);
		} else if (self.shooterJag1.GetSpeed() < self.setPoint) {
			self.shooterJag1.Set(.8);
		}
		*/

	}

	//this is the entire camera task
	static void updateCameraStuff(void *s) {
		
		Threshold threshold(0, 20, 60, 122, 0, 20);
		ParticleFilterCriteria2 criteria[] = { { IMAQ_MT_BOUNDING_RECT_WIDTH,
				30, 400, false, false }, { IMAQ_MT_BOUNDING_RECT_HEIGHT, 40,
				400, false, false } };
/*
		//maybe make the arrays a global constant, so they aren't redeclared each time?
		float distanceTable[] = { 200, 180, 166, 154, 143, 132, 123, 115, 108,102 };
		float rpmTable[] = { 2530, 2520, 2520, 2590, 2650, 2760, 2770, 2820,2920, 3020 };
		*/
		RobotDemo &self = *reinterpret_cast<RobotDemo*> (s);
		
		ColorImage* image = self.camera.GetImage();
		BinaryImage *thresholdImage = image->ThresholdRGB(threshold); // get just the green target pixels
		BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(
				false, 2); // remove small objects (noise)
		BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false); // fill in partial and full rectangles
		BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria,
				2); // find the rectangles
		vector<ParticleAnalysisReport> *reports =
				filteredImage->GetOrderedParticleAnalysisReports(); // get the results			
			for (unsigned i = 0; i < reports->size(); i++) {
			ParticleAnalysisReport *r = &(reports->at(i));
			if (i == 0) {
				self.y1 = r->center_mass_y;
				self.x1 = r->center_mass_x;
			} else if (i == 1) {
				self.y2 = r->center_mass_y;
				self.x2 = r->center_mass_x;
			} else if (i == 2) {
				self.y3 = r->center_mass_y;
				self.x3 = r->center_mass_x;
			} else if (i == 3) {
				self.y4 = r->center_mass_y;
				self.x4 = r->center_mass_x;
			}
			}

		
		self.currentCenter = ((self.x1 + self.x2) / 2) + self.offsetAim;
		if(self.currentCenter - self.centerNeeded < 0)
		{
		 float newSet = fabs(self.currentCenter - self.centerNeeded);
		 self.setValue = -sqrt(newSet);
		}
		else
		{
		self.setValue = sqrt(self.currentCenter - self.centerNeeded);
		}
		self.setValue = self.setValue * .66;
		//self.setValue = (self.currentCenter - self.centerNeeded) / 4;
		//self.setValue = self.setValue/2;
		if (self.setValue > 8) {
			self.setValue = 8;
		} else if (self.setValue < -8) {
			self.setValue = -8;
		}

		if (reports->size() == 0) {
			self.setValue = 0;
		}

		//rpm calculation and interpolation
		/*
		self.currentDistance = self.x2 - self.x1; //distance between the right target - the left target
												  //the higher number the closer we are

		//this if block tests the 2 extreme values we could encounter, 200 being closest.
		if (self.currentDistance > 200) {
			self.shooterSpeed = 2500;
		} else if (self.currentDistance < 102) {
			self.shooterSpeed = 3300;
		}

		//see if we are actually at one of the exact points of data we have
		if (self.currentDistance == 200 || self.currentDistance == 180
				|| self.currentDistance == 166 || self.currentDistance == 154
				|| self.currentDistance == 143 || self.currentDistance == 132
				|| self.currentDistance == 123 || self.currentDistance == 115
				|| self.currentDistance == 108 || self.currentDistance == 102) {
			for (int i = 0; i <= 9; i++) { //if we are at an exact point, see which one and set the RPM accordingly
				if (distanceTable[i] == self.currentDistance) {
					self.shooterSpeed = rpmTable[i];
				}
			}
		}

		//if not at the actual positions, loop through
		float valueToReturn;
		int position;

		for (int i = 0; i <= 9; i++) {
			if ((self.currentDistance <= distanceTable[i])
					&& (self.currentDistance >= distanceTable[i + 1])) {
				//assign a "position" to be used later on
				//200,180,166,154,143,132,123,115,108,102
				position = i;

			}
		}
		
		//here is where position is used, for the linear interp.
		valueToReturn = (((rpmTable[position] - rpmTable[position + 1])
				/ (distanceTable[position] - distanceTable[position + 1]))
				* (self.currentDistance - distanceTable[position + 1]))
				+ rpmTable[position + 1];
		//turretRotate.Set(-setValue);
		*/

		//self.shooterSpeed = fabs(valueToReturn);//take absolute value, to ensure it is always positive...
		//	self.setPoint = -.00009*pow(self.currentDistance,3) + .01097*pow(self.currentDistance,2) - 31.575*self.currentDistance + 5172.4;
		//	printf("Speed: %f Difference: %f \n", self.setPoint, 143);// fabs(self.x2-self.x1));
		//self.updateRotateJag();
		//delete the camera image stuff. Do I need to delete everything else that I declare in this method as well, since it always loops?
		delete reports;
		delete filteredImage;
		delete convexHullImage;
		delete bigObjectsImage;
		delete thresholdImage;
		delete image;
			
		//delete rpmTable;
		//delete distanceTable;
		Wait(.005);
		// printf("Speed: %f ActualSpeed %f  Output: %f \n" , self.setPoint, self.shooterJag1.GetSpeed(), self.shooterJag1.GetOutputVoltage() );
	}

	//currently not using this
	static void printValues(void *s) {
		RobotDemo &self = *reinterpret_cast<RobotDemo*> (s);
		printf("Speed: %f ActualSpeed %f  Output: %f \n", self.setPoint,
				self.shooterJag1.GetSpeed(),
				self.shooterJag1.GetOutputVoltage());
	}

	//will use this on final robot, in conjunction with the limit switch (it works)
	static void checkForButton(void *s) {
		RobotDemo &self = *reinterpret_cast<RobotDemo*> (s);
		if (self.manip.GetRawButton(8)) {
			self.suckingUp = true;
			self.roller1.Set(Relay::kForward);
			self.roller2.Set(Relay::kForward);
			self.turretRoller.Set(Relay::kForward);
			Wait(.75);
			self.turretRoller.Set(Relay::kOff);
			self.roller1.Set(Relay::kOff);
			self.roller2.Set(Relay::kOff);
			self.suckingUp = false;
		}
	}

	//will use this on final robot, in conjunction with the limit switch (it works)
	static void checkIntakes(void *s) {
		RobotDemo &self = *reinterpret_cast<RobotDemo*> (s);
		if (self.intakeSwitch.Get()) {
			self.roller1.Set(Relay::kOff);
			self.roller2.Set(Relay::kOff);
			Wait(.1);
		}
	}

	double absd(double in) {
		if (in < 0)
			return -in;
		return in;
	}

	void updateRotateJag() {
		//setPoint = shooterSpeed; //shooterSpeed is what the camera task updates, but the actual RPM (setPoint) is only updated when we tell it to
		turretRotate.Set(-setValue); //rotate turret head based on tracking data
		//setPoint = giveRPM();
	}

	void Autonomous(void) {
		/*Threshold threshold(0, 30, 24, 117, 0, 30);
		 ParticleFilterCriteria2 criteria[] = { { IMAQ_MT_BOUNDING_RECT_WIDTH,
		 30, 400, false, false }, { IMAQ_MT_BOUNDING_RECT_HEIGHT, 40,
		 400, false, false } };
		 */
		driveTime = true;
		footRight = false;
		footLeft = false;
		suckTime = false;
		shootTime = false;
		ignoreLoop = false;
		goBackwards = false;
		stopDriving = false;
		//currently our autonomous only tracks, i have the rest commented out but it works.
		while (IsAutonomous()) {
			
			shift.Set(true);
			updateRotateJag();

			/*
			updateRotateJag();
			setPoint = giveRPM();*/
		//	printf("General auton \n");
			bridgeManip.Set(true);
			if(!suckTime && !ignoreLoop){
				//printf("Not suck time \n");
			updateRotateJag();
			setPoint = giveRPM(x1,x2);
			//setPoint = 2850;
			suckTime = true;
			 shootTime = false;
			 ignoreLoop = true;
			 for(int i = 0; i < 200; i++)
			 {
					updateRotateJag();
					Wait(.01);
			 }
			 
			 }
			updateRotateJag();

			if(suckTime){

				//printf("Suck time \n");

			//updateRotateJag();
		 //setPoint = giveRPM();
			 suckTime = false;
			 shootTime = true;
			 roller1.Set(Relay::kForward);
			 roller2.Set(Relay::kForward);
			 turretRoller.Set(Relay::kForward);
			 for(int q = 0; q < 210; q++)
			 {
				 updateRotateJag();
				 Wait(.01);
			 }
			 suckTime = false;
			 }

			/*if(suckTime && !driveTime)
			 {
			 Wait(2);
			 driveTime = true;
			 }
			 */
			/*
			 roller1.Set(Relay::kForward);
			 roller2.Set(Relay::kForward);
			 turretRoller.Set(Relay::kForward);
			 */
			 bridgeManip.Set(true);
			 gyroAngle = theGyro.GetAngle();
			// printf("angle %f \n", gyroAngle);
			 
			 double time = GetTime();
			 double left_dist = leftSide.GetDistance();
			 double right_dist = -rightSide.GetDistance();
			 //	double left_rate = (left_dist - leftPrevDist) / (time - leftPrevTime);
			 //	double right_rate = (right_dist - rightPrevDist) / (time - rightPrevTime);
			 leftPrevDist = left_dist;
			 rightPrevDist = right_dist;
			 leftPrevTime = time;
			 rightPrevTime = time;
			 distance = (left_dist + right_dist)/2;
			 
			 if(footRight)
			 {
				 float gyroSetPoint = -5;
				 while(gyroAngle > gyroSetPoint)
				 {
					 printf("Angle: %f \n", gyroAngle);
					 gyroAngle = theGyro.GetAngle();
					 leftJag1.Set(-.8);
					 leftJag2.Set(-.8);
					 rightJag1.Set(-.8);
					 rightJag2.Set(-.8);
				 }
				 footRight = false;
			 }
			 if(footLeft)
			 {
				 float gyroSetPoint = 5;
				 while(gyroAngle < gyroSetPoint)
				 {
					 printf("Angle: %f \n", gyroAngle);
					 gyroAngle = theGyro.GetAngle();
					 leftJag1.Set(.8);
					 leftJag2.Set(.8);
					 rightJag1.Set(.8);
					 rightJag2.Set(.8);
				 }
				 footLeft = false;
			 }
			 
			 
			 if(distance < 7)
			 {
			 keepDriving = true;
			 }
			 else if(distance > 7)
			 {
			 keepDriving = false;
			 }
			 
			 
			 if(distance > 2.5)
			 {
				 driveSpeed = .4;
				 correctSpeed = .65;
				bridgeManip.Set(true);
			 }
			 if(distance > 6.55)
			 {
				 bridgeManip.Set(false);
			 }
			 
			 if(gyroAngle > 1 && keepDriving && !goBackwards)
			 {
			 leftJag1.Set(-correctSpeed);
			 leftJag2.Set(-correctSpeed);
			 rightJag1.Set(driveSpeed);
			 rightJag2.Set(driveSpeed);
			 }
			 else if(gyroAngle < -1 && keepDriving && !goBackwards)
			 {
			 leftJag1.Set(-driveSpeed);
			 leftJag2.Set(-driveSpeed);
			 rightJag1.Set(correctSpeed);
			 rightJag2.Set(correctSpeed);
			 }
			 else if(keepDriving && !goBackwards)
			 {
			 leftJag1.Set(-driveSpeed);
			 leftJag2.Set(-driveSpeed);
			 rightJag1.Set(driveSpeed);
			 rightJag2.Set(driveSpeed);
			 }
			 else if(!keepDriving && !goBackwards && !stopDriving)
			 {
			 leftJag1.Set(-.25);
			 leftJag2.Set(-.25);
			 rightJag1.Set(.25);
			 rightJag2.Set(.25);
			 Wait(1);
			 goBackwards = true;
			 
			 }
			 if(stopDriving)
			 {
				 updateRotateJag();
				 leftJag1.Set(0);
				 leftJag2.Set(0);
				 rightJag1.Set(0);
				 rightJag2.Set(0);
				 for(int g = 0; g < 100; g++)
				 {
					 updateRotateJag();
					 Wait(.01);
				 }
				 setPoint = giveRPM(x1,x2);
				 Wait(.01);
				 turretRoller.Set(Relay::kForward);
			 }
			 
			 
			 if(goBackwards)
			 {
				 turretRotate.Set(0);
				 turretRoller.Set(Relay::kOff);
				 leftJag1.Set(driveSpeed);
				 leftJag2.Set(driveSpeed);
				 rightJag1.Set(-driveSpeed);
				 rightJag2.Set(-driveSpeed);
				 Wait(3);
				 leftJag1.Set(0);
				 leftJag2.Set(0);
				 rightJag1.Set(0);
			     rightJag2.Set(0);
			     stopDriving = true;
			     goBackwards = false;
				 /*
			updateRotateJag();			 
			 turretRoller.Set(Relay::kOff);
				 printf("going backwards \n");
				 if(distance > 8)
				 {
	 				 updateRotateJag();
	 				 printf("distance > 8 \n");
					 if(gyroAngle < 1)
					 			 {
					 			 leftJag1.Set(correctSpeed);
					 			 leftJag2.Set(correctSpeed);
					 			 rightJag1.Set(-driveSpeed);
					 			 rightJag2.Set(-driveSpeed);
					 			 }
					 			 else if(gyroAngle > -1)
					 			 {
					 			 leftJag1.Set(driveSpeed);
					 			 leftJag2.Set(driveSpeed);
					 			 rightJag1.Set(-correctSpeed);
					 			 rightJag2.Set(-correctSpeed);
					 			 }
					 			 else if(keepDriving)
					 			 {
					 			 leftJag1.Set(driveSpeed);
					 			 leftJag2.Set(driveSpeed);
					 			 rightJag1.Set(-driveSpeed);
					 			 rightJag2.Set(-driveSpeed);
					 			 }
					 			 else if(!keepDriving)
					 			 {
					 				 updateRotateJag();
					 			 leftJag1.Set(0);
					 			 leftJag2.Set(0);
					 			 rightJag1.Set(0);
					 			 rightJag2.Set(0);
					 			 bridgeManip.Set(false);
					 			 goBackwards = true;
					 			 }
						stopDriving = true;
						goBackwards = false;
						keepDriving = false;
					
			 }
*/
			 }

			updateRotateJag();
			Wait(.009);
			//setPoint = giveRPM();
		}
	}

	void OperatorControl(void) {
		checkInTake.StartPeriodic(.05);
		checkJoystick.StartPeriodic(.05);
		shift.Set(false);
		while (IsOperatorControl()) {
			
			//printf("TurretPosition: %f \n", turretRotate.GetPosition());
			if(manip.GetTwist() > 0) //down, power saving mode for pushing match
			{
				setPoint = 0;
				turretRoller.Set(Relay::kOff);
				roller1.Set(Relay::kOff);
				roller2.Set(Relay::kOff);
				shooterJag1.Set(0);
			}

			//printf("Setpoint: %f RealSpeed: %f \n", setPoint,shooterJag1.GetSpeed());
			if(manip.GetRawButton(1))
			{
				roller1.Set(Relay::kForward);
				roller2.Set(Relay::kForward);
				turretRoller.Set(Relay::kForward);
			}
			else if(!manip.GetRawButton(1) && !suckingUp)
			{
				turretRoller.Set(Relay::kOff);
				
			}
			
			if(manip.GetRawButton(7) && canSetRPM)
			{
				setPoint = giveRPM(x1,x2);
				canSetRPM = false;
			}
			else if(!manip.GetRawButton(7))
			{
				canSetRPM = true;
			}
			if (manip.GetRawButton(9)) {
				updateRotateJag(); // Aim turret to the target, (works) set RPM accordingly (sort of works)
			}
			else if (manip.GetRawButton(3)) //turn left
			{
				//rotaterSetPoint -= .025;
				turretRotate.Set(12);
			} 
			else if (manip.GetRawButton(4)) //turn right
			{
				//rotaterSetPoint += .025;
				turretRotate.Set(-12);				
			} 
			else
			{
				turretRotate.Set(0);
			}
					

			if (leftStick.GetRawButton(7) || rightStick.GetRawButton(10)) {
				bridgeManip.Set(true); //lower it
			}
			if (leftStick.GetRawButton(6) || rightStick.GetRawButton(11)) {
				bridgeManip.Set(false); //bring it up
			}

			//manually increment the motor setpoint
			if (manip.GetRawButton(6)) {
				setPoint += 10;
			} else if (manip.GetRawButton(5)) {
				setPoint -= 10;
			}

			if (manip.GetRawButton(12)) //suck ball in and up
			{
				roller1.Set(Relay::kForward);
				roller2.Set(Relay::kForward);
				isGoingUp = true;
			}
			if (manip.GetRawButton(11)) //give birth! push out the ball
			{
				roller1.Set(Relay::kReverse);
				roller2.Set(Relay::kReverse);
				isGoingUp = false;
			}
			if (manip.GetRawButton(10)) //Halt, hammerzeit.
			{
				roller2.Set(Relay::kOff);
				roller1.Set(Relay::kOff);
				turretRoller.Set(Relay::kOff);
			}


			//driving code
			leftJag1.Set(leftStick.GetY());
			leftJag2.Set(leftStick.GetY());
			rightJag1.Set(-rightStick.GetY());
			rightJag2.Set(-rightStick.GetY());
			//auto shifting code
			double time = GetTime();
			double left_dist = leftSide.GetDistance();
			double right_dist = -rightSide.GetDistance();
			double left_rate = (left_dist - leftPrevDist) / (time
					- leftPrevTime);
			double right_rate = (right_dist - rightPrevDist) / (time
					- rightPrevTime);
			leftPrevDist = left_dist;
			rightPrevDist = right_dist;
			leftPrevTime = time;
			rightPrevTime = time;
			//printf("left: %f, right: %f, Speed! %f\n", left_rate, right_rate, absd((left_rate + right_rate) / 2));
			if (!HighGear && absd((left_rate + right_rate) / 2) > 3.7) {
				shift.Set(true);
				HighGear = true;
			} else if (HighGear && absd((left_rate + right_rate) / 2) < 2) {
				shift.Set(false);
				HighGear = false;
			}

			//printf("Speed: %f Distance: %f \n" , setPoint, x2-x1);
			dds.sendIOPortData(); //send the relay and digital IO data to the dashboard
			Wait(0.005); //don't overflow the jags, this may need to be set higher

			//setValue = 0;

			//}
		}
	}

};

START_ROBOT_CLASS(RobotDemo)
;

