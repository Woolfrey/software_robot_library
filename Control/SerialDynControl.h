#include <SerialKinControl.h>

class SerialDynControl : public SerialKinControl
{
	public:
		SerialDynControl();
	private:
};										// Semicolon needed after class declaration

SerialDynControl::SerialDynControl()
		:
		SerialKinControl()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}
