#include <SerialKinCtrl.h>

class SerialDynCtrl : public SerialKinCtrl
{
	public:
		SerialDynCtrl();
	private:
};										// Semicolon needed after class declaration

SerialDynCtrl::SerialDynCtrl()
		:
		SerialKinCtrl()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}
