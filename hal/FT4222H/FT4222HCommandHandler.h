/*
 * FT4222HCommandHandler.h
 *
 *  Created on: 13.03.2016
 *      Author: ed / jp
 */

#ifndef FT4222H_COMMAND_HANDLER_H
#define FT4222H_COMMAND_HANDLER_H

#include <QtGui>
#include <QtDebug>
#ifdef _MSC_VER
    #include "stdint.h"
#else
    #include <unistd.h>
#endif

#include <QQueue>
#include <QMutex>

#include "libTMCComm/spi/FT4222H/WeaselDebugInterfaceDefines.h"

#include "FT4222H.h"
#include "CommandHandlerInterface.h"
#include "libTMCCore/tmcl/Command.h"
#include "libTMCCore/tmcl/TMCL.h"
#include "libTMCCore/tmcl/TraceCommand.h"
#include "TMCLRequestCommand.h"
#include "TMCLReplyCommand.h"

#define BLOCKING         true
#define NONE_BLOCKING    false

class FT4222HDataThread;

class TMCCOMM_EXPORT FT4222HCommandHandler : public CommandHandlerInterface
{
    Q_OBJECT
    Q_INTERFACES(CommandHandlerInterface)

    public:
        FT4222HCommandHandler(QObject *parent=nullptr);
        ~FT4222HCommandHandler();

        bool openDevice(QVariant data);
        bool uninitialize();
        bool closeDevice();
        bool reopenDevice();
        bool isDeviceOpen();
        void setDataRate(int m_dataRate);

        QString getFirmwareVersion(quint8 moduleAddress);
        QString getModuleAndFirmwareVersion(quint8 moduleAddress);

        void dumpChipAndLibraryVersion(FT_HANDLE handlePort);
        bool spiSlaveWriteData(quint8* buffer, quint32 numberOfBytes, quint32* numberOfBytesWritten);
        bool spiReset();
        bool GPIOInit(GPIO_Dir gpioDir[4]);
        bool GPIOWrite(GPIO_Port portNum, BOOL pValue);
        bool initializeVhdlModul(RtmiDebuggerDataset * dataset);

        void setICDebuggerDataAvailable(bool enable);
        void clearICDebuggerDataAvailable();

        bool startDebugger();
        bool stopDebugger();
        bool initializeDebugger(RtmiDebuggerDataset * dataset);
        bool initializeBridge(SPI_SlaveProtocol protocolOpt, GPIO_Dir gpioDir[4]);
        bool resetDebugger(void);
        bool isLiveModeActive();

        QQueue<uint8_t> receiveFifo;
        QMutex *interfaceMutex;

        FT_HANDLE handlePortA;      // for SPI-Interface
        FT_HANDLE handlePortB;      // for GPIO

    private:
        bool liveModeActiv = false;
        bool lastLiveModeActiv = false;
        bool debuggerActiv = false;
        long lastSendAddress;
        bool dataReceivedFlag;
        bool dataReadFromBridgeFlag;
        quint8 lastReceivedRegAddr;
        qint32 lastReceivedRegValue;

        bool configDataReadedFlag;
        quint8 lastReadedConfigReg;
        qint32 lastReadedConfigValue;

        void readNextTMCLCommand(TMCLReplyCommand *reply, quint8 command, quint8 lastSendAddress);
        bool waitForReadAccess();
        bool writeTMCLCommand(TMCLRequestCommand *command);
        void setLiveModeActive(bool enable);

        FT4222HDataThread *dataThread;

        QString deviceFlagToString(DWORD flags);
        void listFtUsbDevices();
        RtmiDebuggerDataset icDebuggerDataset;
        bool sendConfigFrame(quint8 paketID, quint8 regAddr, quint32 data, bool transmitMode, quint8 expectedAnswer);
        bool sendRegisterRWFrame(uint8_t paketID, uint8_t motorNr, uint8_t regAddr, uint32_t data);
        bool readConfigRegister(quint8 reg, qint32 readValue);

    public slots:
        void doDirectRequest(Command *request, Command *reply);
        void doDirectRequestWithoutAnswer(Command *request);

    private slots:
        void resetTimeOutedDevices();
        void handleReadPacket(quint8 sync, quint8 id, quint8 timeStamp, qint32 registerValue);

    signals:
        void newChannelData(uint8_t address, uint32_t value);
        void liveDataAvailable(quint8 channelID, quint8 timestamp, qint32 value);
        void liveModeChanged(bool enabled);
};

#endif // FT4222H_COMMANDHANDLER_H
