#include "HQ.h"
#include <ADM.h>

namespace cea
{
	ENCODER::ENCODER()
	{
		mInitialized = false;
		mConfigured = false;
	}
	ENCODER::ENCODER(uint8_t ChanA, void (*chanA_ISR)(void), uint8_t ChanB, void (*chanB_ISR)(void), uint32_t PulsesPerRevolution, char ID[])
	{
		if (!mConfigured)
		{
			for (int i = 0; i < 6; i++)
			{
				if (ID[i] == '\0')
					continue;
				mID[i] = ID[i];
			}
			mCircumference = PulsesPerRevolution;
			mPosition = 0;
			mAchanPin = ChanA;
			mBchanPin = ChanB;
			mChanA_ISR = chanA_ISR;
			mChanB_ISR = chanB_ISR;
			InitIncrement.Register(this);
			FastIncrement.Register(this);
			SlowIncrement.Register(this);
			IncrementMETHOD = &InitIncrement;
			mConfigured = true;
			mInitialized = false;
			//bug << mID << bugVar(mAchanPin) << bugVar(mBchanPin) << "\n";
		}
	}
	bool ENCODER::Initialize(orbos::oPOS& op)
	{
		//xbugWatch(__FUNCTION__);
		if (mConfigured && !mInitialized)
		{
			//bug << mID << bugVar(mAchanPin) << bugVar(mBchanPin) << "\n";
			mPos = &op;
			mCircumference = mPos->Circumference();
			pinMode(mAchanPin, INPUT_PULLUP);
			delay(20);
			pinMode(mBchanPin, INPUT_PULLUP);
			delay(20);
			attachInterrupt(digitalPinToInterrupt(mAchanPin), mChanA_ISR, CHANGE);
			delay(20);
			attachInterrupt(digitalPinToInterrupt(mBchanPin), mChanB_ISR, CHANGE);
			delay(20);
			mConfigured = true;
			return true;
		}
	}
	void ENCODER::TriggerA()
	{
		IncrementMETHOD->IncrementA();		
	}
	void ENCODER::TriggerB()
	{
		IncrementMETHOD->IncrementB();		
	}
	bool ENCODER::UpShiftEncoderCYCLE()
	{
		/* FALSE RETURNS DONE */
		if (mHighSpeed) return false;
		if (!mShiftEncoder) {
			if (mSlowSpeed) {
				mShiftEncoder = true;
				mSlowSpeed = false;
				return true;
			}
			mHighSpeed = true;
			return false;			
		}
		return true;
	}
	bool ENCODER::DownShiftEncoderCYCLE()
	{
		/* FALSE RETURNS DONE */
		if (mSlowSpeed) return false;
		if (!mShiftEncoder) {
			if (mHighSpeed) {
				mShiftEncoder = true;
				mHighSpeed = false;
				return true;
			}
			mSlowSpeed = true;
			return false;
		}
		return true;
	}
	bool ENCODER::EncoderIsUp()
	{
		return (mHighSpeed && !mShiftEncoder);
	}
	bool ENCODER::EncoderIsDown()
	{
		return (mSlowSpeed && !mShiftEncoder);
	}
	volatile uint32_t ENCODER::Position()
	{
		return mPosition;
	}
	double ENCODER::GetPulseCountAndReset()
	{
		double pc = mPulseCount;
		mPulseCount = 0.00f;
		return pc;
	}
	void ENCODER::AnnouncePosition()
	{
		Serial.print("\n\nEncoder Has a Position Recorded of: ");
		Serial.print(mPosition);
		Serial.print("\n\n");
	}
	bool ENCODER::Direction()
	{
		return mDirection;
	}
	void ENCODER::AnnounceDirection()
	{
		Serial.print(F("\n\nDIR -> "));
		Serial.print(F((mDirection ? "Forward" : "Reverse")));
		Serial.print(F("\n\n"));
	}
	bool ENCODER::CalibratePosTo0()
	{
		mPosition = 0;
		return true;
	}

	bool ENCODER::CalibratePosTo(uint32_t calibrateValue)
	{
		if (calibrateValue < mCircumference)
		{
			mPosition = calibrateValue;
			return true;
		}
		return false;
	}

	uint32_t ENCODER::GetPulsesPerRevolution()
	{
		//++(*mPos);
		return mCircumference;
	}
	int ENCODER::GetMissedPulses()
	{
		int missed = mMissedPulses;
		mMissedPulses = 0;
		return missed;
	}
	void SLOWINCREMENT::IncrementA()
	{
		if (Encoder->FlippyFloppy)
		{
			Encoder->mDirection = !Encoder->mDirection;
		}
		if (Encoder->mDirection)
		{
			Encoder->mPos->EncoderIncrementTrigger();
			//Encoder->mPosition = (Encoder->mPosition + 1) % Encoder->mCircumference;
			//Encoder->mPulseCount++;			
		}
		else
		{
			Encoder->mPos->EncoderDecrementTrigger();
			//Encoder->mPosition = ((Encoder->mPosition - 1) + Encoder->mCircumference) % Encoder->mCircumference;
			//Encoder->mPulseCount--;
		}
		Encoder->FlippyFloppy = 1;
		if (Encoder->mShiftEncoder) { Encoder->IncrementMETHOD = &Encoder->FastIncrement; Encoder->mShiftEncoder = false; }

	}
	void SLOWINCREMENT::IncrementB()
	{
		if (!Encoder->FlippyFloppy)
		{
			Encoder->mDirection = !Encoder->mDirection;
		}
		if (Encoder->mDirection)
		{
			Encoder->mPos->EncoderIncrementTrigger();
			//++Encoder->mPos;
			//Encoder->mPosition = (Encoder->mPosition + 1) % Encoder->mCircumference;
			//Encoder->mPulseCount++;
		}
		else
		{
			Encoder->mPos->EncoderDecrementTrigger();
			//--Encoder->mPos;
			//Encoder->mPosition = ((Encoder->mPosition - 1) + Encoder->mCircumference) % Encoder->mCircumference;
			//Encoder->mPulseCount--;
		}
		Encoder->FlippyFloppy = 0;
		if (Encoder->mShiftEncoder) { Encoder->IncrementMETHOD = &Encoder->FastIncrement; Encoder->mShiftEncoder = false; }
	}
	void SLOWINCREMENT::Register(ENCODER* enc)
	{
		Encoder = enc;
	}
	void FASTINCREMENT::IncrementA()
	{	
		/*		Fast Increment assumes constant forward motor 
			direction and so it doesn't do the same direction test
			as the others. However, at highest motor speed,
			it appears that from time to time, either the encoder
			or the arduino misses a pulse. When using a directional
			Algorithm, this translates to a sudden reverse, because
			it appears as if the same pin has fired twice in a row,
			and can dangerously mess up operation and cause logic
			flow to overspeed the motor.
				This fast incrementer solves that issue, and assumes
			that if this channel triggered last, then the other channel
			trigger was missed, so it adds 2 instead, and maintains
			absolute position accuracy after high speed run. */
		if (!Encoder->FlippyFloppy)
		{
			Encoder->mPos->EncoderIncrementTrigger();
			//Encoder->mPosition = (Encoder->mPosition + 1) % Encoder->mCircumference;
			//Encoder->mPulseCount++;
			
		}
		else
		{
			Encoder->mPos->EncoderHighSpeedDoubleIncrement();
			//Encoder->mPosition = (Encoder->mPosition + 2) % Encoder->mCircumference;
			//Encoder->mPulseCount += 2;
			//Encoder->mMissedPulses++;
		}
		Encoder->FlippyFloppy = 1;
		if (Encoder->mShiftEncoder) { Encoder->IncrementMETHOD = &Encoder->SlowIncrement; Encoder->mShiftEncoder = false; }

	}
	void FASTINCREMENT::IncrementB()
	{		
		if (Encoder->FlippyFloppy)
		{
			Encoder->mPos->EncoderIncrementTrigger();
			//Encoder->mPosition = (Encoder->mPosition + 1) % Encoder->mCircumference;
			//Encoder->mPulseCount++;
		}
		else
		{
			Encoder->mPos->EncoderHighSpeedDoubleIncrement();
			//Encoder->mPosition = (Encoder->mPosition + 2) % Encoder->mCircumference;
			//Encoder->mPulseCount += 2;
			//Encoder->mMissedPulses++;
		}
		Encoder->FlippyFloppy = 0;
		if (Encoder->mShiftEncoder) { Encoder->IncrementMETHOD = &Encoder->SlowIncrement; Encoder->mShiftEncoder = false; }
	}
	void FASTINCREMENT::Register(ENCODER* enc)
	{
		Encoder = enc;
	}
	void INITINCREMENT::IncrementA()
	{
		/* This fat and ugly interrupt routine algorithm exists
			to establish a confident direction. Once that is
			established, direction change can be determined
			without any of this, and accurate and absolute
			positioning can be achieved with a very slim 
			algorithm that uses no digital reads. */
		bool A = !digitalRead(Encoder->mAchanPin);
		bool B = !digitalRead(Encoder->mBchanPin);
		bool a = Encoder->mLastA;
		bool b = Encoder->mLastB;
		Encoder->mDirection = (A && !b) || (!A && b);
		if (Encoder->mDirection)
		{
			Encoder->mPos->EncoderIncrementTrigger();			
			//Encoder->mPosition = (Encoder->mPosition + 1) % Encoder->mCircumference;
			//Encoder->mPulseCount++;
		}
		else
		{
			Encoder->mPos->EncoderDecrementTrigger();
			//Encoder->mPosition = ((Encoder->mPosition - 1) + Encoder->mCircumference) % Encoder->mCircumference;
			//Encoder->mPulseCount--;
		}
		Encoder->mStartCounter++;
		Encoder->mLastA = A;
		Encoder->mLastB = B;
		Encoder->FlippyFloppy = 1;
	}
	void INITINCREMENT::IncrementB()
	{
		bool A = !digitalRead(Encoder->mAchanPin);
		bool B = !digitalRead(Encoder->mBchanPin);
		bool a = Encoder->mLastA;
		bool b = Encoder->mLastB;
		Encoder->mDirection = (B && a) || (!B && !a);
		if (Encoder->mDirection)
		{
			Encoder->mPos->EncoderIncrementTrigger();			
			//Encoder->mPosition = (Encoder->mPosition + 1) % Encoder->mCircumference;
			//Encoder->mPulseCount++;
		}
		else
		{
			Encoder->mPos->EncoderDecrementTrigger();
			//Encoder->mPosition = ((Encoder->mPosition - 1) + Encoder->mCircumference) % Encoder->mCircumference;
			//Encoder->mPulseCount--;
		}
		Encoder->mStartCounter++;
		Encoder->mLastA = A;
		Encoder->mLastB = B;
		Encoder->FlippyFloppy = 0;
		if (Encoder->mStartCounter >= Encoder->mPulsesToDirConfirm)
		{			
			/* With Direction being concrete confirmed, we'll quickly shift to a more streamlined low speed increment */
			Encoder->mStartCounter = 0;
			Encoder->IncrementMETHOD = &Encoder->SlowIncrement;	
			Encoder->mSlowSpeed = true;
			return;
		}
	}
	void INITINCREMENT::Register(ENCODER* enc)
	{
		Encoder = enc;
	}
}