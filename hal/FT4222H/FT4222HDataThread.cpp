/*
 * FT4222DataThread.cpp
 *
 *  Created on: 01.09.2016
 *      Author: jp / ed
 */

#include "libTMCComm/spi/FT4222H/FT4222HDataThread.h"

FT4222HDataThread::FT4222HDataThread(FT_HANDLE *ftSPIHandle, QMutex *interfaceMutex, QQueue<uint8_t> *buffer)
{
    m_ftSPIHandle    = ftSPIHandle;
    m_interfaceMutex = interfaceMutex;
    m_receiveBuffer  = buffer;
    m_readDataFromDevice = false;
    m_clearInputBuffer = false;
}

void FT4222HDataThread::run()
{
    long unsigned int numberOfBytesRead;
    uint8_t  buffer[NUMBER_OF_BYTES_PER_DATAFRAME];

    Global::instance()->registerThread(QThread::currentThreadId(), "FT422H");

    qDebug() << __METHOD_NAME__ << "Thread started";

    m_readDataFromDevice = true;

    // Moeglichst hohe Prioritaet, da die USB-SPI-Bridge sonst nicht
    // schnell genug ausgelesen werden kann.
    setPriority(QThread::TimeCriticalPriority);

    while(m_readDataFromDevice)
    {
        // try always to read and use timeout of read function instead of usleep(...)
        m_interfaceMutex->lock();
        //ftd2xx_getQueueStatus(*ftSPIHandle, &numberOfBytesAvailable);

        //if (numberOfBytesAvailable > BUFFER_LENGHT)
        //    numberOfBytesAvailable = BUFFER_LENGHT;

        //if (numberOfBytesAvailable < NUMBER_OF_BYTES_PER_DATAFRAME)
        //    numberOfBytesAvailable = NUMBER_OF_BYTES_PER_DATAFRAME;

        ftd2xx_read(*m_ftSPIHandle, buffer, NUMBER_OF_BYTES_PER_DATAFRAME, &numberOfBytesRead);
        m_interfaceMutex->unlock();

        if (numberOfBytesRead > 0)
        {
            // read all available data
            for(quint32 i = 0; i < numberOfBytesRead; i++)
                m_receiveBuffer->enqueue(buffer[i]);

            if (m_clearInputBuffer)
            {
                m_clearInputBuffer = false;
                m_receiveBuffer->clear();
            }

            //qDebug() << __METHOD_NAME__ << "NoBytes:" << QString::number(numberOfBytesRead)  << "(" << QString::number(receiveBuffer->size()) << ")";

            while (m_receiveBuffer->length() >= NUMBER_OF_BYTES_PER_DATAFRAME)
            {
                quint8 sync = m_receiveBuffer->dequeue();
                if(sync == SYNCWORD)
                {
                   quint8 id = m_receiveBuffer->dequeue();            // packetID + channelID
                   quint8 timeStamp = m_receiveBuffer->dequeue();     // timestamp

                   // register data
                   qint32  registerValue;
                   registerValue = m_receiveBuffer->dequeue() << 24;  // data, msb first
                   registerValue |= m_receiveBuffer->dequeue() << 16; // data
                   registerValue |= m_receiveBuffer->dequeue() << 8;  // data
                   registerValue |= m_receiveBuffer->dequeue();       // data

                   emit newReadPacket(sync, id, timeStamp, registerValue);
                }
                else
                {
                    qDebug() << __METHOD_NAME__ << "Missing sync word! "
                                                << "SyncWord: " << QString::number(sync)
                                                << "BufferLen:" << QString::number(m_receiveBuffer->length() + 1);
                }
            }
        }
    }

    Global::instance()->unregisterThread(QThread::currentThreadId());
}

void FT4222HDataThread::stopReadingData()
{
    m_readDataFromDevice = false;
}

bool FT4222HDataThread::state()
{
    return m_readDataFromDevice;
}

void FT4222HDataThread::clearQueue()
{
    m_clearInputBuffer = true;
}
