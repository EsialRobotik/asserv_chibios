#include "CommandHandler.h"
#include "AsservMain.h"
#include "commandManager/CommandManager.h"

// #include <ch.h>
// #include <hal.h>
// #include <chprintf.h>


CommandHandler::CommandHandler(CommandManager & cmdManager, AsservMain & asservMain )
: m_cmdManager(cmdManager), m_asservMain(asservMain)
{
}


// extern BaseSequentialStream *outputStream;

void CommandHandler::applyCommand(QCBORDecodeContext &decodeCtx)
{
	QCBORDecode_EnterMap(&decodeCtx, NULL);
		
	UsefulBufC name;
	QCBORDecode_GetTextStringInMapSZ(&decodeCtx, "name", &name);

	if ( strncmp((char*)name.ptr, "reset" ,strlen("reset")) == 0 )
	{
		// chprintf(outputStream, "Cmd: reset\r\n");
		 
		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_asservMain.reset();
	}
	else if ( strncmp((char*)name.ptr, "enablemotor" ,strlen("enablemotor")) == 0 )
	{
		// chprintf(outputStream, "Cmd: enable motor\r\n");
		
		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_asservMain.enableMotors(true);
	}
	else if ( strncmp((char*)name.ptr, "disablemotor" ,strlen("disablemotor")) == 0 )
	{
		// chprintf(outputStream, "Cmd: disable motor\r\n");
		
		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_asservMain.enableMotors(false);
	}
	else if ( strncmp((char*)name.ptr, "robotfwspeedstep" ,strlen("robotfwspeedstep")) == 0 )
	{
		double speed, duration;
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "speed", &speed);
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "duration", &duration);

		// chprintf(outputStream, "Cmd: forward speed step speed %f duration %f\r\n", speed, duration);

		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_cmdManager.addWheelsSpeed(speed, speed, duration);
	}
	else if ( strncmp((char*)name.ptr, "robotangspeedstep" ,strlen("robotangspeedstep")) == 0 )
	{
		double speed, duration;
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "speed", &speed);
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "duration", &duration);


		// chprintf(outputStream, "Cmd: angle speed step speed %f duration %f\r\n", speed, duration);

		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
		{
			if( speed > 0)
				m_cmdManager.addWheelsSpeed(speed/2, -speed/2, duration);
			else
				m_cmdManager.addWheelsSpeed(-speed/2, speed/2, duration);
		}
	}
	else if ( strncmp((char*)name.ptr, "wheelspeedstepLeft" ,strlen("wheelspeedstepLeft")) == 0 )
	{
		double speed, duration;
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "speed", &speed);
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "duration", &duration);

		// chprintf(outputStream, "Cmd: Left wheel speed step speed %f duration %f\r\n", speed, duration);

		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_cmdManager.addWheelsSpeed(0, speed, duration);
	}
	else if ( strncmp((char*)name.ptr, "wheelspeedstepRight" ,strlen("wheelspeedstepRight")) == 0 )
	{
		double speed, duration;
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "speed", &speed);
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "duration", &duration);

		// chprintf(outputStream, "Cmd: Right wheel speed step speed %f duration %f\r\n", speed, duration);

		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_cmdManager.addWheelsSpeed(speed, 0, duration);
	}
	else if ( strncmp((char*)name.ptr, "adddist" ,strlen("adddist")) == 0 )
	{
		double dist;
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "dist", &dist);

		// chprintf(outputStream, "Cmd: add distance %f\r\n", dist);

		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_cmdManager.addStraightLine(dist);
	}
	else if ( strncmp((char*)name.ptr, "addangle" ,strlen("addangle")) == 0 )
	{
		double angle;
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "angle", &angle);

		// chprintf(outputStream, "Cmd: add angle %f\r\n", angle);

		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_cmdManager.addTurn(angle);
	}
	else if ( strncmp((char*)name.ptr, "addgoto" ,strlen("addgoto")) == 0 )
	{
		double X,Y;
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "X", &X);
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "Y", &Y);

		// chprintf(outputStream, "Cmd: add goto (%f, %f)\r\n", X,Y);

		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_cmdManager.addGoTo(X,Y);
	}
	else if ( strncmp((char*)name.ptr, "addgotoNoStop" ,strlen("addgotoNoStop")) == 0 )
	{
		double X,Y;
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "X", &X);
		QCBORDecode_GetDoubleInMapSZ(&decodeCtx, "Y", &Y);

		// chprintf(outputStream, "Cmd: add goto nostop (%f, %f)\r\n", X,Y);

		if( QCBORDecode_GetError(&decodeCtx) == QCBOR_SUCCESS)
			m_cmdManager.addGoToNoStop(X,Y);
	}	

	
	
	QCBORDecode_ExitMap(&decodeCtx);
}