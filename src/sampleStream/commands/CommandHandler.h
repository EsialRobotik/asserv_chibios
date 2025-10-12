#ifndef COMMAND_HANDLER_H_
#define COMMAND_HANDLER_H_

#include "qcbor/qcbor_decode.h"
#include "qcbor/qcbor_spiffy_decode.h"

class CommandManager;
class AsservMain;

class CommandHandler
{
public:
	explicit CommandHandler(CommandManager & cmdManager, AsservMain & asservMain );

	void applyCommand(QCBORDecodeContext &decodeCtx);


private:

	CommandManager & m_cmdManager;
	AsservMain & m_asservMain;

};

#endif /* COMMAND_HANDLER_H_ */
