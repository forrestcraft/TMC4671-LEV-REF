/*
 * FT4222DataThread.h
 *
 *  Created on: 01.09.2016
 *      Author: jp, ed
 */

#ifndef FT4222H_DATA_THREAD_H
#define FT4222H_DATA_THREAD_H

#include <QDebug>
#include <QThread>
#include <QMutex>
#include <QQueue>

#include "FT4222HCommandHandler.h"

class FT4222HDataThread : public QThread
{
    Q_OBJECT

    public:
        FT4222HDataThread(FT_HANDLE *ftSPIHandle, QMutex *interfaceMutex, QQueue<uint8_t> *buffer);
        void stopReadingData();
        bool state();
        void clearQueue();

    private:
        bool m_readDataFromDevice;
        bool m_clearInputBuffer;
        void run();
        FT_HANDLE *m_ftSPIHandle;
        QMutex *m_interfaceMutex;
        QQueue<uint8_t> *m_receiveBuffer;

    signals:
        void newReadPacket(quint8 sync, quint8 id, quint8 timeStamp, qint32 registerValue);
};

#endif // FT4222H_DATA_THREAD_H

