#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <WebServer.h>
#endif

enum class CommunicationCommandType
{
	None,
	SetControlMode,
	SetInteractionMode,
	SetWorldTarget,
	NudgeWorld,
	NudgeJoint,
	SelectAutomaticSlot
};

struct CommunicationCommand
{
	CommunicationCommandType type;
	bool manualMode;
	bool worldMode;
	float x;
	float y;
	float z;
	char axis;
	int direction;
	int joint;
	int slot;
};

struct CommunicationStatus
{
	bool manualMode;
	bool worldMode;
	bool reachable;
	float x;
	float y;
	float z;
	float baseAngle;
	float joint1Angle;
	float joint2Angle;
	int selectedSlot;
};

class Communication
{
public:
	Communication();

	void begin();
	void handleClient();
	bool nextCommand(CommunicationCommand &command);
	void setStatus(const CommunicationStatus &status);

private:
	CommunicationCommand m_pendingCommand;
	CommunicationStatus m_status;
	bool m_hasPendingCommand;

#if defined(ARDUINO_ARCH_ESP32)
	WebServer m_server;
	void handleCommand();
	void enqueue(const CommunicationCommand &command);
#endif
};
