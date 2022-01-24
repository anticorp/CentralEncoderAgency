#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <ClockCache.h>
#include "OrbitalPositionLibrary.h"

namespace cea
{
	class ENCODER;
	class INCREMENTMETHOD
	{
	public:
		virtual void IncrementA() {}
		virtual void IncrementB() {}
		virtual void Register(ENCODER*) {}		
	protected:
		ENCODER* Encoder;
		volatile int MissedPulseOffset = 0;
		//volatile orbos::ORBITALPOSITION 

	};
	class INITINCREMENT : public INCREMENTMETHOD
	{
	public:
		void IncrementA() override;
		void IncrementB() override;
		void Register(ENCODER*) override;
	};
	class FASTINCREMENT : public INCREMENTMETHOD
	{
	public:
		void IncrementA() override;
		void IncrementB() override;
		void Register(ENCODER*) override;
	};

	class SLOWINCREMENT : public INCREMENTMETHOD
	{
	public:
		void IncrementA() override;
		void IncrementB() override;
		void Register(ENCODER*) override;
	};
	class ENCODER
	{
	public:
		ENCODER();
		ENCODER(uint8_t ChanA, void (*chanA_ISR)(void), uint8_t ChanB, void (*chanB_ISR)(void), uint32_t PulsesPerRevolution, char ID[]);
		bool Initialize(orbos::oPOS& op);
		void TriggerA();
		void TriggerB();
		virtual void IncrementA() {}
		virtual void IncrementB() {}
		virtual void Register(ENCODER*) {}
		/*	Returns FALSE once done. Waits until position change, then shifts the encoder speed pointers. This is to avoid an interrupt happening right in the middle of a shift.   */
		bool UpShiftEncoderCYCLE();
		/*	Returns FALSE once done. Waits until position change, then shifts the encoder speed pointers. This is to avoid an interrupt happening right in the middle of a shift.   */
		bool DownShiftEncoderCYCLE();
		/*	CURRENT POSITION RELATIVE TO ZERO */
		bool EncoderIsUp();
		bool EncoderIsDown();
		volatile uint32_t Position();
		/* Returns a raw Pulse Incremented/decremented value and resets the counter to 0. For velocity measurments to avoid doing math/conditionals around the 0 degree rollover mark */
		double GetPulseCountAndReset();
		/*	Sends Serial Output of Current Position */
		void AnnouncePosition();
		/* FOWARD = TRUE  */
		bool Direction();
		void AnnounceDirection();
		bool CalibratePosTo0();
		bool CalibratePosTo(uint32_t calibrateValue);
		uint32_t GetPulsesPerRevolution();
		int GetMissedPulses();

	protected:
		friend class FASTINCREMENT;
		friend class SLOWINCREMENT;
		friend class INITINCREMENT;
		volatile bool mShiftEncoder = false;
		bool mHighSpeed = false;
		bool mSlowSpeed = false;
		bool mInitialized = false;
		bool mConfigured = false;
		bool mPosInit = false;
		void (*mChanA_ISR)(void);
		void (*mChanB_ISR)(void);
		INCREMENTMETHOD* IncrementMETHOD = nullptr;
		FASTINCREMENT FastIncrement;
		SLOWINCREMENT SlowIncrement;
		INITINCREMENT InitIncrement;
		volatile uint32_t mCircumference;
		uint32_t mAchanPin;
		uint32_t mBchanPin;
		volatile bool dotheupdate;
		orbos::oPOS* mPos;
		volatile int32_t mPosition;
		volatile double mPulseCount = 0.00f;
		volatile int mMissedPulses = 0;
		volatile int FlippyFloppy = 0;
		volatile bool mLastA;
		volatile bool mLastB;
		volatile bool mDirection;
		volatile int PosCache = -1;
		volatile int mStartCounter = 0;
		volatile bool Update = false;
		const int mPulsesToDirConfirm = 2;
		char mID[6];
	};

	


}