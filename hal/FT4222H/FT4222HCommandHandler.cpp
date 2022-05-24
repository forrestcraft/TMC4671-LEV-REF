/*
 * FT4222HCommandHandler.cpp
 *
 *  Created on: 13.03.2016
 *      Author: ed / jp
 */

#include "libTMCComm/spi/FT4222H/FT4222HCommandHandler.h"
#include "libTMCComm/spi/FT4222H/FT4222HDataThread.h"

// FPGA registeraddresses
#define DEVICE_INFO_REGISTER                    0x00
#define CHANNEL_0_REGISTER                      0x01
#define CHANNEL_1_REGISTER                      0x02
#define CHANNEL_2_REGISTER                      0x03
#define CHANNEL_3_REGISTER                      0x04
#define GENERAL_CONFIG_REGISTER                 0x05
#define TRIGGERVALUE_REGISTER                   0x06
#define VALUES_PER_SINGLE_SHOT_REGISTER         0x07

FT4222HCommandHandler::FT4222HCommandHandler(QObject *parent) : CommandHandlerInterface(parent)
{
    m_dataRate = 1000;
    setMaxTimeoutMS(100);
    setDefaultTimeoutMS(50);
    setObjectName("SPI FT4222H");
    setDeviceName(objectName());
    handlePortA = nullptr;
    handlePortB = nullptr;
    dataReceivedFlag = false;
    dataReadFromBridgeFlag = false;
    setLiveModeActive(false);
    debuggerActiv = false;
    configDataReadedFlag = true;

    interfaceMutex  = new QMutex(QMutex::NonRecursive);
    dataThread      = new FT4222HDataThread(&handlePortA, interfaceMutex, &receiveFifo);
    QObject::connect(dataThread, SIGNAL(newReadPacket(quint8, quint8, quint8, qint32)), this, SLOT(handleReadPacket(quint8, quint8, quint8, qint32)), /*Qt::QueuedConnection*/Qt::DirectConnection);
}

FT4222HCommandHandler::~FT4222HCommandHandler()
{
    dataThread->stopReadingData();
    dataThread->wait(1000);

    if (dataThread->isFinished())
    {
        qDebug() << __METHOD_NAME__ << "dataThread finished.";
        delete dataThread;
    }
//    else
//        qDebug() << __METHOD_NAME__ << "dataThread not finished!";

    //delete receiveFifo;
    //delete pufferMutex;
    delete interfaceMutex;
    FT4222H_unloadLibraries();
}

void FT4222HCommandHandler::dumpChipAndLibraryVersion(FT_HANDLE handlePort)
{
    FT4222_Version version;
    FT4222_STATUS status = ft4222_getVersion(handlePort, &version);
    if (status == FT4222_OK)
        qDebug() << __METHOD_NAME__ << "LibFT4222: chipVersion:" << BasicTools::iToHex((quint64)version.chipVersion)
                                    << "dllVersion:" << BasicTools::iToHex((quint64)version.dllVersion);
    else
        qDebug() << __METHOD_NAME__ << "LibFT422: unable to read chip and library version!";
}

bool FT4222HCommandHandler::openDevice(QVariant data)
{
    Q_UNUSED(data);

    qDebug() << __METHOD_NAME__ << deviceName();

    m_open = false;
    m_timeoutedAddresses.clear();

    // Mode 2: für 2xSPI-Slave + 2xIO-Pin
    // a. The first 3 interfaces: SPI master connects up to 3 SPI slaves.
    // b. The 4th interface: GPIO device.

    // try to load FT4222H library
    if (FT4222H_loadLibraries())
    {
        listFtUsbDevices();

        // create the device information list
        DWORD numOfDevices;
        FT_STATUS status = ftd2xx_createDeviceInfoList(&numOfDevices);

        qDebug() << __METHOD_NAME__ << " Devices: " << QString::number(numOfDevices);

        FT_DEVICE_LIST_INFO_NODE devInfo;
        QString desc;

        for(DWORD iDev = 0; iDev < numOfDevices; ++iDev)
        {

            memset(&devInfo, 0, sizeof(devInfo));

            // get the device information list
            status = ftd2xx_getDeviceInfoDetail(iDev, &devInfo.Flags, &devInfo.Type, &devInfo.ID, &devInfo.LocId,
                                                devInfo.SerialNumber, devInfo.Description, &devInfo.ftHandle);

            if (FT_OK == status)
            {
                qDebug() << __METHOD_NAME__ << "   Dev:    " << QString::number(iDev);
                qDebug() << __METHOD_NAME__ << "   Desc:   " << devInfo.Description;
                qDebug() << __METHOD_NAME__ << "   Flags:  " << deviceFlagToString(devInfo.Flags) << " (" << QString::number(devInfo.Flags) << ")";
                qDebug() << __METHOD_NAME__ << "   Type:   " << QString::number(devInfo.Type);
                qDebug() << __METHOD_NAME__ << "   ID:     " << QString::number(devInfo.ID);
                qDebug() << __METHOD_NAME__ << "   LocId:  " << QString::number(devInfo.LocId);
                qDebug() << __METHOD_NAME__ << "   SerNr.: " << devInfo.SerialNumber;

                desc = devInfo.Description;

                if(desc == "FT4222 A")
                {
                    if (ftd2xx_openEx((PVOID)devInfo.LocId, FT_OPEN_BY_LOCATION, &handlePortA) == FT_OK)
                    {
                        qDebug() << __METHOD_NAME__ << "opened handlePortA:" << desc;
                        ftd2xx_setTimeouts(handlePortA, 1/*00*/, 100);
                    }
                    else
                    {
                        qDebug() << __METHOD_NAME__ << "Open FT4222_A device failed!";
                        return false;
                    }
                }
                dumpChipAndLibraryVersion(handlePortA);

                if(desc == "FT4222 B")
                {
                    if (ftd2xx_openEx((PVOID)devInfo.LocId, FT_OPEN_BY_LOCATION, &handlePortB) == FT_OK)
                    {
                        qDebug() << __METHOD_NAME__ << "opened handlePortB:" << desc;
                        ftd2xx_setTimeouts(handlePortB, 1/*00*/, 100);
                    }
                    else
                    {
                        qDebug() << __METHOD_NAME__ << "Open FT4222_B device failed!";
                        return false;
                    }
                }
            }
            else
            {
                qDebug() << __METHOD_NAME__ << "state of ftd2xx_getDeviceInfoDetail:" << QString::number(status);
            }
        }
    } else {
        qDebug() << __METHOD_NAME__ << "initialisation failed!!!";
        return false;
    }

    // reseting the Bridge
    resetDebugger();

    // set system clock to 80MHz
    FT_STATUS status = ft4222_setClock(handlePortA, SYS_CLK_80);
    qDebug() << __METHOD_NAME__ << "Status of ft4222_setClock:" << QString::number(status);

    // Initialize the Bridge
    GPIO_Dir gpioDir[4] = {GPIO_OUTPUT, GPIO_OUTPUT, GPIO_OUTPUT, GPIO_OUTPUT};

    if(initializeBridge(SPI_SLAVE_NO_PROTOCOL, gpioDir))
        qDebug() << __METHOD_NAME__ << "Initialize Bridge --> OK";
    else{
        qDebug() << __METHOD_NAME__ << "Initialize Bridge --> FAILED";
        return false;
    }

    // starting the receive thread
    dataThread->start();
    m_open = true;
    return true;
}

bool FT4222HCommandHandler::isDeviceOpen()
{
    return m_open;
}

bool FT4222HCommandHandler::GPIOInit(GPIO_Dir gpioDir[4])
{
    FT_STATUS state =  ft4222_GPIO_Init(handlePortB, gpioDir);

    qDebug() << __METHOD_NAME__ << "state of ft4222_GPIO_Init:" << QString::number(state);

    if(state != FT4222_OK)
        return false;

    // set GPIOs to zero
    GPIOWrite(GPIO_PORT0, false);
    GPIOWrite(GPIO_PORT1, false);
    GPIOWrite(GPIO_PORT2, false);
    GPIOWrite(GPIO_PORT3, false);
    return true;
}

//bool FT4222HCommandHandler::GPIORead(GPIO_Port portNum, BOOL* pValue)
//{
//    qDebug() << __METHOD_NAME__;

//    int state =  ft4222_GPIO_Read(handlePortB, portNum, pValue);

//    if(state != FT4222_OK)
//        return false;

//    return true;
//}

bool FT4222HCommandHandler::GPIOWrite(GPIO_Port portNum, BOOL pValue)
{
    if (handlePortB)
        return (ft4222_GPIO_Write(handlePortB, portNum, pValue) == FT4222_OK);

    return false;
}

bool FT4222HCommandHandler::uninitialize()
{
    int state =  ft4222_UnInitialize(handlePortB);

    qDebug() << __METHOD_NAME__ << state;

    if(state != FT4222_OK)
        return false;

    return true;
}

bool FT4222HCommandHandler::initializeVhdlModul(RtmiDebuggerDataset *dataset)
{
    qDebug() << __METHOD_NAME__;

    // falls nach dem Stop noch Live-Daten empfangen wurden
    // Problem existiert in FT4222HQ Rev C
    setLiveModeActive(false);

    qDebug() << __METHOD_NAME__ << "a)";

    // Channel 0 Register
    quint32 tmpRegister = (dataset->channelReg_Enable[0]<<31)
                | (dataset->channelReg_SubAddrEnable[0]<<30)
                | (dataset->channelReg_MotorNr[0]<<24)
                | (dataset->channelReg_SubAddress[0]<<16)
                | (dataset->channelReg_SubAddressValue[0]<<8)
                | dataset->channelReg_Address[0];

    qDebug() << __METHOD_NAME__ << "b)";

    do{
        qDebug() << __METHOD_NAME__ << "Channel 0 Register:" << QString::number(tmpRegister,16);
        if(sendConfigFrame(PACKET_ID_WRITE_CONFIG, CHANNEL_0_REGISTER, tmpRegister, BLOCKING, 1) == false)
            return false;
    }while(!readConfigRegister(CHANNEL_0_REGISTER, tmpRegister));

    qDebug() << __METHOD_NAME__ << "c)";

    // Channel 1 Register
    tmpRegister = (dataset->channelReg_Enable[1]<<31)
            | (dataset->channelReg_SubAddrEnable[1]<<30)
            | (dataset->channelReg_MotorNr[1]<<24)
            | (dataset->channelReg_SubAddress[1]<<16)
            | (dataset->channelReg_SubAddressValue[1]<<8)
            | dataset->channelReg_Address[1];

    qDebug() << __METHOD_NAME__ << "d)";
    do{
        qDebug() << __METHOD_NAME__ << "Channel 1 Register:" << QString::number(tmpRegister,16);
        if(sendConfigFrame(PACKET_ID_WRITE_CONFIG, CHANNEL_1_REGISTER, tmpRegister, BLOCKING, 1) == false)
            return false;
    }while(!readConfigRegister(CHANNEL_1_REGISTER, tmpRegister));

    qDebug() << __METHOD_NAME__ << "e)";

    // Channel 2 Register
    tmpRegister = (dataset->channelReg_Enable[2]<<31)
            | (dataset->channelReg_SubAddrEnable[2]<<30)
            | (dataset->channelReg_MotorNr[2]<<24)
            | (dataset->channelReg_SubAddress[2]<<16)
            | (dataset->channelReg_SubAddressValue[2]<<8)
            | dataset->channelReg_Address[2];

    qDebug() << __METHOD_NAME__ << "f)";

    do{
        qDebug() << __METHOD_NAME__ << "Channel 2 Register:" << QString::number(tmpRegister,16);
        if(sendConfigFrame(PACKET_ID_WRITE_CONFIG, CHANNEL_2_REGISTER, tmpRegister, BLOCKING, 1) == false)
            return false;
    }while(!readConfigRegister(CHANNEL_2_REGISTER, tmpRegister));

    qDebug() << __METHOD_NAME__ << "g)";

    // Channel 3 Register
    tmpRegister = (dataset->channelReg_Enable[3]<<31)
            | (dataset->channelReg_SubAddrEnable[3]<<30)
            | (dataset->channelReg_MotorNr[3]<<24)
            | (dataset->channelReg_SubAddress[3]<<16)
            | (dataset->channelReg_SubAddressValue[3]<<8)
            | dataset->channelReg_Address[3];

    qDebug() << __METHOD_NAME__ << "h)";

    do{
        qDebug() << __METHOD_NAME__ << "Channel 3 Register:" << QString::number(tmpRegister,16);
        if(sendConfigFrame(PACKET_ID_WRITE_CONFIG, CHANNEL_3_REGISTER, tmpRegister, BLOCKING, 1) == false)
            return false;
    }while(!readConfigRegister(CHANNEL_3_REGISTER, tmpRegister));

    qDebug() << __METHOD_NAME__ << "i)";

    // General Config Register
    tmpRegister = dataset->gcr_liveMode;
    tmpRegister |= 8<<1;             // Wait 8 clock-cycles after write register
    tmpRegister |= dataset->gcr_triggerChannel<<8;
    tmpRegister |= dataset->gcr_triggerMode<<12;
    tmpRegister |= dataset->gcr_downsampling<<16;

    qDebug() << __METHOD_NAME__ << "j)";

    do{
        qDebug() << __METHOD_NAME__ << "General Config Register:" << QString::number(tmpRegister,16);
        if(sendConfigFrame(PACKET_ID_WRITE_CONFIG, GENERAL_CONFIG_REGISTER, tmpRegister, BLOCKING, 1) == false)
            return false;
    }while(!readConfigRegister(GENERAL_CONFIG_REGISTER, tmpRegister));

    qDebug() << __METHOD_NAME__ << "k)";

    // Trigger Value
    tmpRegister = dataset->triggerValue;

    do{
        qDebug() << __METHOD_NAME__ << "TriggerValue:" << QString::number(tmpRegister,16);
        if(sendConfigFrame(PACKET_ID_WRITE_CONFIG, TRIGGERVALUE_REGISTER, tmpRegister, BLOCKING, 1) == false)
            return false;
    }while(!readConfigRegister(TRIGGERVALUE_REGISTER, tmpRegister));

    qDebug() << __METHOD_NAME__ << "l)";

    // VALUES_PER_SINGLE_SHOT_REGISTER
    tmpRegister = dataset->valuesPerShot;

    do{
        qDebug() << __METHOD_NAME__ << "Singleshot Values Register:" << QString::number(tmpRegister,16);
        if(sendConfigFrame(PACKET_ID_WRITE_CONFIG, VALUES_PER_SINGLE_SHOT_REGISTER, tmpRegister, BLOCKING, 1) == false)
            return false;
    }while(!readConfigRegister(VALUES_PER_SINGLE_SHOT_REGISTER, tmpRegister));

    qDebug() << __METHOD_NAME__ << "m)";

    return true; // TODO
}

bool FT4222HCommandHandler::readConfigRegister(quint8 reg, qint32 regValue)
{
    if(sendConfigFrame(PACKET_ID_READ_CONFIG, reg, 0x00, BLOCKING, 1))
    {
        QElapsedTimer timeoutTimer;
        timeoutTimer.start();

        // check if command was readed
        while (true)
        {
            if(configDataReadedFlag)
            {
                configDataReadedFlag = false;

                if((reg == lastReadedConfigReg) && (regValue == lastReadedConfigValue))
                    return true;
                else{
                    qDebug() << __METHOD_NAME__ << "(reg == lastReadedConfigReg) && (regValue == lastReadedConfigValue)!";
                    return false;
                }
            }

            usleep(25); // 1ms waiting is too long!

            if (timeoutTimer.elapsed() >= m_maxTimeoutMS)
            {
                configDataReadedFlag = false;
                qDebug() << __METHOD_NAME__ << "communicationTimeouted!";

                if (m_timeoutedAddresses.isEmpty())
                    QTimer::singleShot(5000, this, SLOT(resetTimeOutedDevices()));

                if (!m_timeoutedAddresses.contains(static_cast<quint8>(lastSendAddress)))
                    m_timeoutedAddresses.append(static_cast<quint8>(lastSendAddress));

                return false;
            }
        }
    }
    // send datagram failed
    resetDebugger();

    return false;
}

bool FT4222HCommandHandler::isLiveModeActive()
{
    return liveModeActiv;
}

void FT4222HCommandHandler::setLiveModeActive(bool enable)
{
    liveModeActiv = enable;

    if (lastLiveModeActiv != liveModeActiv)
    {
        qCDebug(debLowLevel) << Q_FUNC_INFO << enable;
        emit liveModeChanged(enable);
    }

    lastLiveModeActiv = liveModeActiv;
}

void FT4222HCommandHandler::setICDebuggerDataAvailable(bool enable)
{
    GPIOWrite(GPIO_DATA_AVAILABLE, enable);
}

void FT4222HCommandHandler::clearICDebuggerDataAvailable()
{
    GPIOWrite(GPIO_DATA_AVAILABLE, false);
}

// start the live monitoring
// it is necessary that the Debugger was initialized before starting the live monitoring.
bool FT4222HCommandHandler::startDebugger()
{
    // wird true sobald die ersten Live-Daten empfangen wurden
    setLiveModeActive(false);
    debuggerActiv = true;

    //start measure
    if(sendConfigFrame(PACKET_ID_START_MEASURE, 0x00, 0x00000000, NONE_BLOCKING, 1))
    {
        qDebug() << __METHOD_NAME__ << "Start Debugger --> OK";
        return true;
    }
    else
    {
        qDebug() << __METHOD_NAME__ << "Start Debugger --> FAILED";
        return false;
    }
}

// stopp the live monitoring
bool FT4222HCommandHandler::stopDebugger()
{
    qDebug() << __METHOD_NAME__;

    // stop measurement
    if(sendConfigFrame(PACKET_ID_STOP_MEASURE, 0x00, 0x00000000, NONE_BLOCKING, 1))
    {
        qDebug() << __METHOD_NAME__ << "Stop Debugger --> OK";

        // wird true sobald die ersten Live-Daten empfangen wurden
        setLiveModeActive(false);
        debuggerActiv = false;
        return true;
    }
    else
    {
        qDebug() << __METHOD_NAME__ << "Stop Debugger --> FAILED";
        setLiveModeActive(false);
        debuggerActiv = false;
        return false;
    }
}

// initialize the VHDL Modud in the FPGA/IC and the bridge
bool FT4222HCommandHandler::initializeDebugger(RtmiDebuggerDataset *dataset)
{
    // initialize the VHDL modul
    if(initializeVhdlModul(dataset))
        qDebug() << __METHOD_NAME__ << "Initialize VHDL Modul --> OK";
    else
    {
        qDebug() << __METHOD_NAME__ << "Initialize VHDL Modul --> FAILED";
        return false;
    }
    return true;
}

// tnitialize the mode and gpios of the FT4222H
bool FT4222HCommandHandler::initializeBridge(SPI_SlaveProtocol protocolOpt, GPIO_Dir gpioDir[4])
{
    // init gpios as output
    if(GPIOInit(gpioDir))
        qDebug() << __METHOD_NAME__ << "GPIO Init --> OK";
    else
    {
        qDebug() << __METHOD_NAME__ << "GPIO Init --> FAILED";
        return false;
    }

    // init bridge as slave
    if (ft4222_SPISlave_InitEx(handlePortA, protocolOpt) == FT4222_OK)
        qDebug() << __METHOD_NAME__ << "SPI Slave Init --> OK";
    else
    {
        qDebug() << __METHOD_NAME__ << "SPI Slave Init --> FAILED";
        return false;
    }
    return true;
}

bool FT4222HCommandHandler::resetDebugger(void)
{
    qDebug() << __METHOD_NAME__;

    // reset the FT4222 Modul
    interfaceMutex->lock();
    int state = ft4222_SPI_Reset(handlePortA);
    interfaceMutex->unlock();
    return (state == FT4222_OK);
}

bool FT4222HCommandHandler::spiSlaveWriteData(quint8 *buffer, quint32 numberOfBytes, quint32 *numberOfBytesWritten)
{
    interfaceMutex->lock();
    int state = ftd2xx_write(handlePortA, buffer, (unsigned long int)numberOfBytes, (unsigned long int*)numberOfBytesWritten);
    interfaceMutex->unlock();

    if(state != FT4222_OK)
    {
        qDebug() << __METHOD_NAME__ << QString::number(state) << QString::number(numberOfBytes) << QString::number(*numberOfBytesWritten);
        return false;
    }
    return true;
}

bool FT4222HCommandHandler::spiReset()
{
     qDebug() << __METHOD_NAME__;

    interfaceMutex->lock();
    int state = ft4222_SPI_Reset(handlePortA);
    interfaceMutex->unlock();

    if(state != FT4222_OK)
        return false;

    return true;
}

bool FT4222HCommandHandler::reopenDevice()
{
    qDebug() << __METHOD_NAME__ << deviceName();
    return false;
}

void FT4222HCommandHandler::setDataRate(int dataRate)
{
    Q_UNUSED(dataRate);
}

bool FT4222HCommandHandler::closeDevice()
{
    dataThread->stopReadingData();

//    if (debug)
        qDebug() << __METHOD_NAME__ << deviceName();

    interfaceMutex->lock();

    if (handlePortA)
    {
        if (ftd2xx_close(handlePortA) == FT_OK)
        {
            qDebug() << __METHOD_NAME__ << "handlePortA closed.";
            handlePortA = nullptr;
        } else {
            qDebug() << __METHOD_NAME__ << "Could not close handlePortA!";
        }
    }

    if (handlePortB)
    {
        if (ftd2xx_close(handlePortB) == FT_OK)
        {
            qDebug() << __METHOD_NAME__ << "handlePortB closed.";
            handlePortB = nullptr;
        } else {
            qDebug() << __METHOD_NAME__ << "Could not close handlePortB!";
        }
    }

    interfaceMutex->unlock();

    FT4222H_unloadLibraries();
    m_open = false;
    return true;
}

QString FT4222HCommandHandler::getFirmwareVersion(quint8 moduleAddress)
{
    Q_UNUSED(moduleAddress);
    qDebug() << __METHOD_NAME__ << deviceName();
    return QString();
}

QString FT4222HCommandHandler::getModuleAndFirmwareVersion(quint8 moduleAddress)
{
    Q_UNUSED(moduleAddress);
    qDebug() << __METHOD_NAME__ << deviceName();
    return "";
}

void FT4222HCommandHandler::doDirectRequest(Command *request, Command *reply)
{
    //qDebug() << __METHOD_NAME__;

    if (liveModeActiv)
    {
        //reply is invalid!
        if (reply != nullptr)
            reply->setValid(false);

        return;
    }

    TMCLRequestCommand actualRequest(request->address(), request->command(), request->type(), request->motorBank(), request->value());
    if(writeTMCLCommand(&actualRequest))
    {
        TMCLReplyCommand actualReply;
        readNextTMCLCommand(&actualReply, request->command(), request->address());

        if(m_currentHistoryEnabled)
            emit variantMessageOutput(Serial_TMCL_Reply, QVariant::fromValue(&actualReply));

        if(actualReply.isValid() && (reply != nullptr))
        {
            // "return" valid reply
            reply->setValid(true);
            reply->setAddress(request->address());
            reply->setCommand(request->command());
            reply->setType(actualReply.type()/*request->getType()*/);

            if (actualReply.type() != request->type())
            {
                dataThread->clearQueue();
//                qDebug() << __METHOD_NAME__ << "type mismatch: requestType=" << QString::number(request->type())
//                                                             << "replyType=" << QString::number(reply->type());
            }

            reply->setStatus(actualReply.status());
            reply->setMotorBank(request->motorBank());
            reply->setValue(actualReply.value());
            reply->setReplyAddress(actualReply.replyAddress());
            //reply->setChecksum(actualReply.getCheckSum());
            //qDebug() << __METHOD_NAME__ << QString::number(reply->value) << QString::number(actualReply.getValue());
            // variable for checksum validity is missing in the "Command"! (ED)
        }
        else
        {
            //reply is invalid!
            if (reply != nullptr)
                reply->setValid(false);
        }
    } else {
        //reply is invalid!
        if (reply != nullptr)
            reply->setValid(false);

        if(m_currentHistoryEnabled)
        {
            TMCLReplyCommand reply;
            emit variantMessageOutput(Serial_TMCL_Reply, QVariant::fromValue(&reply));
        }

        if (m_timeoutedAddresses.isEmpty())
            QTimer::singleShot(3000, this, SLOT(resetTimeOutedDevices()));

        if (!m_timeoutedAddresses.contains(static_cast<quint8>(lastSendAddress)))
            m_timeoutedAddresses.append(static_cast<quint8>(lastSendAddress));
    }
}
#define _Test_8Bit
bool FT4222HCommandHandler::writeTMCLCommand(TMCLRequestCommand *command)
{
    if (m_debug)
    {
        qDebug() << __METHOD_NAME__ << command->toString();
        qDebug() << __METHOD_NAME__ << command->toHex();
    }

    if(m_currentHistoryEnabled)
        emit variantMessageOutput(Serial_TMCL_Request, QVariant::fromValue(command));

    lastSendAddress = command->moduleAddress();

    if (m_open && dataThread->state() && !m_timeoutedAddresses.contains(static_cast<quint8>(lastSendAddress)))
    {
        if(((command->command()) == TMCL_EVAL_ReadChannelRegister_MC)
        || ((command->command()) == TMCL_EVAL_ReadChannelRegister_DRV))
        {
#ifdef _Test_8Bit
            return sendRegisterRWFrame(PACKET_ID_READ_REGISTER, command->motorNumber(), ((command->type() & 0xFF)), command->value());
#else
            return sendRegisterRWFrame(PACKET_ID_READ_REGISTER, command->motorNumber(), ((command->type() & 0x7F)), command->value());
#endif
        }
        else if(((command->command()) == TMCL_EVAL_WriteChannelRegister_MC)
             || ((command->command()) == TMCL_EVAL_WriteChannelRegister_DRV))
        {
#ifdef _Test_8Bit
            return sendRegisterRWFrame(PACKET_ID_WRITE_REGISTER, command->motorNumber(), ((command->type() & 0xFF)), command->value());
#else
            return sendRegisterRWFrame(PACKET_ID_WRITE_REGISTER, command->motorNumber(), ((command->type() & 0x7F)), command->value());
#endif
        }
        else
            qDebug() << __METHOD_NAME__ << "Wrong command:" << QString::number(command->command()) << "!";
    }
    return false;
}

void FT4222HCommandHandler::readNextTMCLCommand(TMCLReplyCommand *reply, quint8 command, quint8 lastSendAddress)
{
    // is the device open and no timeout ?
    if (!m_open || m_timeoutedAddresses.contains(lastSendAddress))
    {
        if (m_debug) qDebug() << __METHOD_NAME__ << "!open";
        reply->setValid(false);
        return;
    }

    QElapsedTimer timeoutTimer;
    timeoutTimer.start();

    while (true)
    {
        // Bei einem Lesebefehl wird eine Antwort erwartet, bei einem Schreibbefehl nicht
        if((command == TMCL_EVAL_ReadChannelRegister_MC) || (command == TMCL_EVAL_ReadChannelRegister_DRV))
        {
            if(dataReceivedFlag)
            {
                dataReceivedFlag = false;
                reply->setValid(true);
                reply->setType(lastReceivedRegAddr);
                reply->setValue(lastReceivedRegValue);
                reply->setStatus(REPLY_OK);
            }
        }
        else if((command == TMCL_EVAL_WriteChannelRegister_MC) || (command == TMCL_EVAL_WriteChannelRegister_DRV))
        {
            reply->setValid(true);
        }

        if (reply->isValid())
        {
            if (m_debug)
            {
                qDebug() << __METHOD_NAME__ << reply->toHex();
                qDebug() << __METHOD_NAME__ << reply->toString();
            }
            return;
        }

        //usleep(25); // 1ms waiting is too long!

        if (timeoutTimer.elapsed() >= m_maxTimeoutMS)
        {
            qDebug() << __METHOD_NAME__ << "communicationTimeouted!";

            if (m_timeoutedAddresses.isEmpty())
                QTimer::singleShot(3000, this, SLOT(resetTimeOutedDevices()));

            if (!m_timeoutedAddresses.contains(lastSendAddress))
                m_timeoutedAddresses.append(lastSendAddress);

            reply->setValid(false);
            return;
        }
    }
    reply->setValid(false);
    return;
}

void FT4222HCommandHandler::resetTimeOutedDevices()
{
    qDebug() << __METHOD_NAME__;
    m_timeoutedAddresses.clear();
}

void FT4222HCommandHandler::handleReadPacket(quint8 sync, quint8 id, quint8 timeStamp, qint32 registerValue)
{
    //qDebug() << __METHOD_NAME__ << "Sync:" << QString::number(sync);

    if(sync == SYNCWORD)
    {
        switch(id & PACKET_ID_MASK)
        {
                // live data
            case PACKET_ID_LIVE_DATA:
                // switch to live mode
               if(debuggerActiv == true)
                    setLiveModeActive(true);
                    //liveModeActiv = true;

                emit liveDataAvailable(id & CHANNEL_ID_MASK, timeStamp, registerValue);
                incTmclCommandCounter();
                break;

                // register access
            case PACKET_ID_READ_REGISTER:
                // register address, value
                dataReceivedFlag = true;
                lastReceivedRegAddr = timeStamp;
                lastReceivedRegValue = registerValue;
                incTmclCommandCounter();
                break;

                // konfiguration data
            case PACKET_ID_READ_CONFIG:
                // register address, value
                configDataReadedFlag = true;
                lastReadedConfigReg = timeStamp;
                lastReadedConfigValue = registerValue;
                incTmclCommandCounter();
                break;

                // read acces from FPGA to usb-bridge
            case PACKET_ID_DUMMY:
                // packet has been received from the bridge!?
                dataReadFromBridgeFlag = true;
                break;
            default:
                qDebug() << __METHOD_NAME__ << "Unknow paket ID:" << QString::number(id & PACKET_ID_MASK);
                break;
        }
    }
}

bool FT4222HCommandHandler::waitForReadAccess()
{
    QElapsedTimer timeoutTimer;
    timeoutTimer.start();

    while (true)
    {
        if(dataReadFromBridgeFlag)
        {
            dataReadFromBridgeFlag = false;
            return true;
        }

        //usleep(25);

        if (timeoutTimer.elapsed() >= m_maxTimeoutMS)
        {
            dataReadFromBridgeFlag = false;
            qDebug() << __METHOD_NAME__ << "communication timeouted!";
            return false;
        }
    }
    return false;
}

void FT4222HCommandHandler::doDirectRequestWithoutAnswer(Command *request)
{
    Q_UNUSED(request);
    qDebug() << __METHOD_NAME__ << "not implemented!";
}

QString FT4222HCommandHandler::deviceFlagToString(DWORD flags)
{
    QString msg;
    msg.append((flags & 0x1) ? "OPEN" : "CLOSED");
    msg.append(", ");
    msg.append((flags & 0x2) ? "High-speed" : "Full-speed");
    return msg;
}

void FT4222HCommandHandler::listFtUsbDevices()
{
    DWORD numDevs;
    FT_STATUS ftStatus = ftd2xx_createDeviceInfoList(&numDevs);

    // dump devices
    if(numDevs > 0)
    {
        // allocate storage for list based on numDevs

        //TODO: speicher auch wieder freigeben!!! (ED)
        FT_DEVICE_LIST_INFO_NODE *devInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE)*numDevs);

        // get the device information list
        ftStatus = ftd2xx_getDeviceInfoList(devInfo, &numDevs);

        if (ftStatus == FT_OK)
        {
            qDebug() << __METHOD_NAME__ << " Devices: " << QString::number(numDevs);
            for (int i = 0; i < (int)numDevs; i++)
            {
                qDebug() << __METHOD_NAME__ << "   Desc:   " << QString(devInfo[i].Description);
                qDebug() << __METHOD_NAME__ << "   Flags:  " << deviceFlagToString(devInfo[i].Flags) << " (" << QString::number(devInfo[i].Flags) << ")";
                qDebug() << __METHOD_NAME__ << "   Type:   " << QString::number(devInfo[i].Type);
                qDebug() << __METHOD_NAME__ << "   ID:     " << QString::number(devInfo[i].ID);
                qDebug() << __METHOD_NAME__ << "   LocId:  " << QString::number(devInfo[i].LocId);
                qDebug() << __METHOD_NAME__ << "   SerNr.: " << devInfo[i].SerialNumber;
            }
        }
    }
}

bool FT4222HCommandHandler::sendConfigFrame(quint8 paketID, quint8 regAddr, quint32 data, bool transmitMode, quint8 expectedAnswer)
{
    Q_UNUSED(transmitMode);
    Q_UNUSED(expectedAnswer);
    quint8     buffer[NUMBER_OF_BYTES_PER_DATAFRAME+1];
    quint32    numbersOfBytesWritten;

    qDebug() << __METHOD_NAME__;
    setICDebuggerDataAvailable(false);

    buffer[0] = 0x00;
    buffer[1] = SYNCWORD;
    buffer[2] = paketID;
    buffer[3] = regAddr;
    buffer[4] = (uint8_t)((data & 0xFF000000) >> 24);
    buffer[5] = (uint8_t)((data & 0x00FF0000) >> 16);
    buffer[6] = (uint8_t)((data & 0x0000FF00) >> 8);
    buffer[7] = (uint8_t)(data & 0x000000FF);

    int attempt = 0;
    do{
        bool state;

        if(liveModeActiv)
            state = spiSlaveWriteData(buffer, NUMBER_OF_BYTES_PER_DATAFRAME+1, &numbersOfBytesWritten);
        else
            state = spiSlaveWriteData(&buffer[1], NUMBER_OF_BYTES_PER_DATAFRAME, &numbersOfBytesWritten);

        if(state == false)
        {
            qDebug() << __METHOD_NAME__ << "--> Write failed!";
            return false;
        }
        setICDebuggerDataAvailable(false);
        setICDebuggerDataAvailable(true);
        setICDebuggerDataAvailable(false);
        setICDebuggerDataAvailable(true);

        attempt++;

        if (attempt > 1)
            qDebug() << __METHOD_NAME__ << "retry:" << QString::number(attempt);

        // abort after 5 attempts
        if(attempt>=5)
        {
            qDebug() << __METHOD_NAME__ << "configuration --> failed!";
            return false;
        }
    }
    while(!waitForReadAccess());

    return true;
}

bool FT4222HCommandHandler::sendRegisterRWFrame(uint8_t paketID, uint8_t motorNr, uint8_t regAddr, uint32_t data)
{
    quint8      buffer[NUMBER_OF_BYTES_PER_DATAFRAME+1];
    quint32     numbersOfBytesWritten;

    //qDebug() << __METHOD_NAME__;
    setICDebuggerDataAvailable(false);

    buffer[0] = 0x00;
    buffer[1] = SYNCWORD;
    buffer[2] = paketID | motorNr;
    buffer[3] = regAddr;
    buffer[4] = (uint8_t)((data & 0xFF000000) >> 24);
    buffer[5] = (uint8_t)((data & 0x00FF0000) >> 16);
    buffer[6] = (uint8_t)((data & 0x0000FF00) >> 8);
    buffer[7] = (uint8_t)(data & 0x000000FF);

    int attempt = 0;
    do{
        bool state;

        if(liveModeActiv)
            state = spiSlaveWriteData(buffer, NUMBER_OF_BYTES_PER_DATAFRAME+1, &numbersOfBytesWritten);
        else
            state = spiSlaveWriteData(&buffer[1], NUMBER_OF_BYTES_PER_DATAFRAME, &numbersOfBytesWritten);

        if(state == false)
        {
            qDebug() << __METHOD_NAME__ << "--> write failed!";
            return false;
        }

        setICDebuggerDataAvailable(false);
        setICDebuggerDataAvailable(true);
        setICDebuggerDataAvailable(false);        // dirty fix for Type-C variant!
        setICDebuggerDataAvailable(true);

        attempt++;

        if (attempt > 1)
            qDebug() << __METHOD_NAME__ << "retry:" << QString::number(attempt);

        // abort after 5 attempts
        if(attempt >= 5)
        {
            qDebug() << __METHOD_NAME__ << "communication --> failed!";
            return false;
        }
    }
    while(!waitForReadAccess());

    incTmclCommandCounter();
    return true;
}
