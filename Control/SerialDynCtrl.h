#include <SerialKinCtrl.h>

class SerialDynCtrl : public SerialKinCtrl
{
	public:
		SerialDynCtrl(const SerialLink &serial) : SerialKinCtrl(serial) {}	// Constructor
		
	private:
};											// Semicolon needed after class declaration
