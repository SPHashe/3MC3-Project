//Unfortunately, this is all AI, I dont know how to write this just yet, but I will liekly erase all of this an write it myself

#include "Communication.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <math.h>

namespace
{
	const char AccessPointName[] = "3MC3-Robot-Arm";
	const char AccessPointPassword[] = "robotarm3mc3";

	const char ControlPage[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
	<meta charset="utf-8">
	<meta name="viewport" content="width=device-width, initial-scale=1">
	<title>3MC3 Arm Control</title>
	<style>
		:root{color-scheme:dark;--bg:#101514;--panel:#19211f;--line:#34413c;--text:#eef4ef;--muted:#a5b3aa;--green:#4cdb88;--red:#ff6868;--lime:#c7f36b;--button:#27332e}
		*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:16px/1.45 system-ui,sans-serif}
		main{max-width:980px;margin:auto;padding:20px}header{display:flex;align-items:center;justify-content:space-between;gap:16px;border-bottom:1px solid var(--line);padding:8px 0 18px}
		h1{font-size:22px;margin:0}h2{font-size:16px;margin:0 0 14px}.muted{color:var(--muted);font-size:13px}
		button,input{font:inherit;color:var(--text);border:1px solid var(--line);border-radius:6px;background:var(--button);min-height:44px;padding:9px 12px}
		button{cursor:pointer}button:hover:enabled{border-color:var(--lime)}button:disabled,input:disabled{opacity:.42;cursor:not-allowed}
		.mode{display:flex;gap:8px}.active{border-color:var(--lime);color:var(--lime)}section{padding:20px 0;border-bottom:1px solid var(--line)}
		.row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}.status{margin-left:auto;color:var(--muted)}.dot{width:12px;height:12px;border-radius:50%;background:var(--red);display:inline-block;margin-right:7px}.dot.ok{background:var(--green)}
		.world-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;max-width:430px}.world-grid button{font-weight:650}
		.coords{display:grid;grid-template-columns:repeat(3,minmax(95px,1fr));gap:9px;margin:14px 0;max-width:430px}.coords label{display:grid;gap:5px;color:var(--muted);font-size:13px}.coords input{width:100%}
		.joints{display:grid;grid-template-columns:repeat(2,minmax(180px,1fr));gap:12px;max-width:540px}.joint{display:flex;justify-content:space-between;align-items:center;border:1px solid var(--line);padding:12px;border-radius:6px}
		.slots{display:grid;grid-template-columns:repeat(3,minmax(70px,100px));gap:9px}.slots button{aspect-ratio:1;font-size:20px}.slots button.selected{background:var(--green);color:#102117;border-color:var(--green)}
		[hidden]{display:none!important}
		@media(max-width:600px){main{padding:14px}header{align-items:flex-start;flex-direction:column}.status{margin-left:0}.joints{grid-template-columns:1fr}.coords{max-width:none}.world-grid{max-width:none}}
	</style>
</head>
<body><main>
	<header><div><h1>3MC3 Arm</h1><div class="muted">Local control panel</div></div><div class="row"><button id="controlMode">Loading mode...</button><span class="status" id="connection">Connecting</span></div></header>
	<section><div class="row"><h2>Manual control mode</h2><div class="mode"><button id="worldMode">World</button><button id="jointMode">Joint</button></div></div><div class="muted" id="manualHint">World and joint controls are available in Manual mode.</div></section>
	<section id="worldPanel"><div class="row"><h2>World coordinates</h2><span class="status"><i id="reachableDot" class="dot"></i><span id="reachableText">No target</span></span></div>
		<div class="world-grid"><button data-axis="x" data-dir="1">X+</button><button data-axis="y" data-dir="1">Y+</button><button data-axis="z" data-dir="1">Z+</button><button data-axis="x" data-dir="-1">X-</button><button data-axis="y" data-dir="-1">Y-</button><button data-axis="z" data-dir="-1">Z-</button></div>
		<div class="coords"><label>X (mm)<input id="x" type="number" step="1"></label><label>Y (mm)<input id="y" type="number" step="1"></label><label>Z (mm)<input id="z" type="number" step="1"></label></div>
		<button id="goToTarget">Move to XYZ</button>
	</section>
	<section id="jointPanel" hidden><h2>Joint control</h2><div class="joints">
		<div class="joint"><span>Joint 1 <strong id="joint1Angle">--</strong> deg</span><button data-joint="1">+1 deg</button></div>
		<div class="joint"><span>Joint 2 <strong id="joint2Angle">--</strong> deg</span><button data-joint="2">+1 deg</button></div>
	</div></section>
	<section id="autoPanel" hidden><h2>Automatic positions</h2><div class="slots" id="slots"></div><div class="muted" style="margin-top:10px">Selected position turns green.</div></section>
	<section><h2>Joint angles</h2><div class="muted" id="angles">Base -- deg · Joint 1 -- deg · Joint 2 -- deg</div></section>
</main>
<script>
const $=id=>document.getElementById(id);let state={manual:false,world:false,selectedSlot:-1,x:0,y:0,z:0};
async function send(data){try{await fetch('/api/cmd?'+new URLSearchParams(data));await update()}catch(e){$('connection').textContent='Disconnected'}}
function render(){let manual=state.manual; $('controlMode').textContent=manual?'Manual mode':'Automatic mode';$('controlMode').classList.toggle('active',manual);$('worldMode').classList.toggle('active',state.world);$('jointMode').classList.toggle('active',!state.world);$('worldPanel').hidden=!manual||!state.world;$('jointPanel').hidden=!manual||state.world;$('autoPanel').hidden=manual;$('worldMode').disabled=!manual;$('jointMode').disabled=!manual;$('manualHint').textContent=manual?'Manual controls active':'Choose Manual to enable World and Joint controls';document.querySelectorAll('[data-axis],[data-joint],#goToTarget').forEach(b=>b.disabled=!manual);document.querySelectorAll('.slots button').forEach((b,i)=>{b.disabled=manual;b.classList.toggle('selected',i===state.selectedSlot)});for(const k of ['x','y','z'])$(k).value=Number(state[k]).toFixed(0)}
async function update(){try{let r=await fetch('/api/status');state=await r.json();$('connection').textContent='Connected';$('joint1Angle').textContent=Number(state.joint1).toFixed(1);$('joint2Angle').textContent=Number(state.joint2).toFixed(1);$('angles').textContent=`Base ${Number(state.base).toFixed(1)} deg · Joint 1 ${Number(state.joint1).toFixed(1)} deg · Joint 2 ${Number(state.joint2).toFixed(1)} deg`;$('reachableDot').classList.toggle('ok',state.reachable);$('reachableText').textContent=state.reachable?'Reachable':'Unreachable';render()}catch(e){$('connection').textContent='Disconnected'}}
$('controlMode').onclick=()=>send({type:'control',manual:state.manual?'0':'1'});$('worldMode').onclick=()=>send({type:'mode',world:'1'});$('jointMode').onclick=()=>send({type:'mode',world:'0'});
document.querySelectorAll('[data-axis]').forEach(b=>b.onclick=()=>send({type:'nudge',axis:b.dataset.axis,dir:b.dataset.dir}));document.querySelectorAll('[data-joint]').forEach(b=>b.onclick=()=>send({type:'joint',joint:b.dataset.joint,dir:'1'}));
$('goToTarget').onclick=()=>send({type:'target',x:$('x').value,y:$('y').value,z:$('z').value});
const slots=$('slots');for(let i=0;i<9;i++){let b=document.createElement('button');b.textContent=i+1;b.onclick=()=>send({type:'slot',slot:i});slots.appendChild(b)}
update();setInterval(update,500);
</script></body></html>
)rawliteral";
}
#endif

Communication::Communication()
	: m_hasPendingCommand(false)
{
	m_pendingCommand.type = CommunicationCommandType::None;
	m_status.manualMode = false;
	m_status.worldMode = false;
	m_status.reachable = false;
	m_status.x = 0;
	m_status.y = 0;
	m_status.z = 0;
	m_status.baseAngle = 0;
	m_status.joint1Angle = 0;
	m_status.joint2Angle = 0;
	m_status.selectedSlot = -1;
}

void Communication::begin()
{
#if defined(ARDUINO_ARCH_ESP32)
	WiFi.mode(WIFI_AP);
	WiFi.softAP(AccessPointName, AccessPointPassword);

	m_server.on("/", HTTP_GET, [this]() {
		m_server.send_P(200, "text/html", ControlPage);
	});
	m_server.on("/api/status", HTTP_GET, [this]() {
		String json = "{\"manual\":" + String(m_status.manualMode ? "true" : "false");
		json += ",\"world\":" + String(m_status.worldMode ? "true" : "false");
		json += ",\"reachable\":" + String(m_status.reachable ? "true" : "false");
		json += ",\"x\":" + String(m_status.x, 1);
		json += ",\"y\":" + String(m_status.y, 1);
		json += ",\"z\":" + String(m_status.z, 1);
		json += ",\"base\":" + String(m_status.baseAngle, 1);
		json += ",\"joint1\":" + String(m_status.joint1Angle, 1);
		json += ",\"joint2\":" + String(m_status.joint2Angle, 1);
		json += ",\"selectedSlot\":" + String(m_status.selectedSlot) + "}";
		m_server.send(200, "application/json", json);
	});
	m_server.on("/api/cmd", HTTP_GET, [this]() { handleCommand(); });
	m_server.onNotFound([this]() { m_server.send(404, "text/plain", "Not found"); });
	m_server.begin();

	Serial.print("Arm access point: ");
	Serial.println(AccessPointName);
	Serial.print("Control page: http://");
	Serial.println(WiFi.softAPIP());
#endif
}

void Communication::handleClient()
{
#if defined(ARDUINO_ARCH_ESP32)
	m_server.handleClient();
#endif
}

bool Communication::nextCommand(CommunicationCommand &command)
{
	if (!m_hasPendingCommand)
	{
		return false;
	}

	command = m_pendingCommand;
	m_hasPendingCommand = false;
	return true;
}

void Communication::setStatus(const CommunicationStatus &status)
{
	m_status = status;
}

#if defined(ARDUINO_ARCH_ESP32)
void Communication::enqueue(const CommunicationCommand &command)
{
	if (!m_hasPendingCommand)
	{
		m_pendingCommand = command;
		m_hasPendingCommand = true;
	}
}

void Communication::handleCommand()
{
	CommunicationCommand command = {};
	String type = m_server.arg("type");

	if (type == "control")
	{
		command.type = CommunicationCommandType::SetControlMode;
		command.manualMode = m_server.arg("manual") == "1";
	}
	else if (type == "mode")
	{
		command.type = CommunicationCommandType::SetInteractionMode;
		command.worldMode = m_server.arg("world") == "1";
	}
	else if (type == "target" && m_server.hasArg("x") && m_server.hasArg("y") && m_server.hasArg("z"))
	{
		command.type = CommunicationCommandType::SetWorldTarget;
		command.x = m_server.arg("x").toFloat();
		command.y = m_server.arg("y").toFloat();
		command.z = m_server.arg("z").toFloat();
		if (!isfinite(command.x) || !isfinite(command.y) || !isfinite(command.z))
		{
			m_server.send(400, "text/plain", "Invalid XYZ values");
			return;
		}
	}
	else if (type == "nudge")
	{
		String axis = m_server.arg("axis");
		if (axis.length() != 1 || (axis[0] != 'x' && axis[0] != 'y' && axis[0] != 'z'))
		{
			m_server.send(400, "text/plain", "Invalid axis");
			return;
		}
		command.type = CommunicationCommandType::NudgeWorld;
		command.axis = axis[0];
		command.direction = m_server.arg("dir").toInt() < 0 ? -1 : 1;
	}
	else if (type == "joint")
	{
		command.type = CommunicationCommandType::NudgeJoint;
		command.joint = m_server.arg("joint").toInt();
		command.direction = m_server.arg("dir").toInt() < 0 ? -1 : 1;
		if (command.joint < 1 || command.joint > 2)
		{
			m_server.send(400, "text/plain", "Invalid joint");
			return;
		}
	}
	else if (type == "slot")
	{
		command.type = CommunicationCommandType::SelectAutomaticSlot;
		command.slot = m_server.arg("slot").toInt();
		if (command.slot < 0 || command.slot > 8)
		{
			m_server.send(400, "text/plain", "Invalid slot");
			return;
		}
	}
	else
	{
		m_server.send(400, "text/plain", "Unknown command");
		return;
	}

	enqueue(command);
	m_server.send(202, "text/plain", "Accepted");
}
#endif
