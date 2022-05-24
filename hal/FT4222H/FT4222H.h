/*
 * FT222H.h
 *
 *  Created on: 13.03.2016
 *      Author: ed
 */

#ifndef FT222H_H
#define FT222H_H

#include <QDebug>

// include FTDI libraries
#include "ftd2xx.h"
#include "LibFT4222.h"

// type declarations for the function pointer into ftd2xx.dll
typedef FT_STATUS (__stdcall FT_CreateDeviceInfoList)(LPDWORD lpdwNumDevs);
typedef FT_STATUS (__stdcall FT_GetDeviceInfoList)(FT_DEVICE_LIST_INFO_NODE *pDest, LPDWORD lpdwNumDevs);
typedef FT_STATUS (__stdcall FT_GetDeviceInfoDetail)(DWORD dwIndex, LPDWORD lpdwFlags, LPDWORD lpdwType,
                                                        LPDWORD lpdwID, LPDWORD lpdwLocId, LPVOID lpSerialNumber,
                                                        LPVOID lpDescription, FT_HANDLE *pftHandle);
typedef FT_STATUS (__stdcall FT_OpenEx)(PVOID pArg1, DWORD Flags, FT_HANDLE *pHandle);
typedef FT_STATUS (__stdcall FT_Close)(FT_HANDLE ftHandle);
typedef FT_STATUS (__stdcall FT_SetTimeouts)(FT_HANDLE ftHandle, ULONG readTimeout, ULONG writeTimeout);
typedef FT_STATUS (__stdcall FT_Read)(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD dwBytesToRead, LPDWORD lpdwBytesReturned);
typedef FT_STATUS (__stdcall FT_Write)(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD dwBytesToWrite, LPDWORD lpdwBytesWritten);
typedef FT_STATUS (__stdcall FT_GetQueueStatus)(FT_HANDLE ftHandle, LPDWORD lpdwBytesReturned);
typedef FT_STATUS (__stdcall FT_GetStatus)(FT_HANDLE ftHandle, LPDWORD lpdwAmountInRxQueue, LPDWORD lpdwAmountInTxQueue, LPDWORD lpdwEventStatus);

typedef FT_STATUS (__stdcall FT_GetDriverVersion)(FT_HANDLE ftHandle, LPDWORD lpdwVersion);
typedef FT_STATUS (__stdcall FT_GetLibraryVersion)(LPDWORD lpdwVersion);

// function pointer into the ftd2xx dll
extern FT_CreateDeviceInfoList  *ftd2xx_createDeviceInfoList;
extern FT_GetDeviceInfoList     *ftd2xx_getDeviceInfoList;
extern FT_GetDeviceInfoDetail   *ftd2xx_getDeviceInfoDetail;
extern FT_OpenEx                *ftd2xx_openEx;
extern FT_Close                 *ftd2xx_close;
extern FT_SetTimeouts           *ftd2xx_setTimeouts;
extern FT_Read                  *ftd2xx_read;
extern FT_Write                 *ftd2xx_write;
extern FT_GetQueueStatus        *ftd2xx_getQueueStatus;
extern FT_GetStatus             *ftd2xx_getStatus;
extern FT_GetDriverVersion      *ftd2xx_getDriverVersion;
extern FT_GetLibraryVersion     *ftd2xx_getLibraryVersion;

// type declarations for the function pointer into libFT4222.dll
typedef FT4222_STATUS (__stdcall FT_4222_UnInitialize)(FT_HANDLE pHandle);
typedef FT4222_STATUS (__stdcall FT_4222_SetClock)(FT_HANDLE pHandle, FT4222_ClockRate clk);
typedef FT4222_STATUS (__stdcall FT_4222_GetClock)(FT_HANDLE pHandle, FT4222_ClockRate* pClk);
typedef FT4222_STATUS (__stdcall FT_4222_GetVersion)(FT_HANDLE pHandle, FT4222_Version* pVersion);
typedef FT4222_STATUS (__stdcall FT_4222_GetMaxTransferSize)(FT_HANDLE pHandle, uint16* pMaxSize);
typedef FT4222_STATUS (__stdcall FT_4222_GPIO_Init)(FT_HANDLE ftHandle, GPIO_Dir gpioDir[4]);
typedef FT4222_STATUS (__stdcall FT_4222_GPIO_Read)(FT_HANDLE ftHandle, GPIO_Port portNum, BOOL* pValue);
typedef FT4222_STATUS (__stdcall FT_4222_GPIO_Write)(FT_HANDLE ftHandle, GPIO_Port portNum, BOOL bValue);

typedef FT4222_STATUS (__stdcall FT_4222_SPIMaster_Init)(FT_HANDLE ftHandle, FT4222_SPIMode ioLine,
                                                        FT4222_SPIClock clock_div, FT4222_SPICPOL cpol,
                                                        FT4222_SPICPHA cpha, uint8 ssoMap);
typedef FT4222_STATUS (__stdcall FT_4222_SPIMaster_SingleRead)(FT_HANDLE ftHandle, uint8* buffer,
                                                        uint16 bytesToRead, uint16* sizeOfRead,
                                                        BOOL isEndTransaction);
typedef FT4222_STATUS (__stdcall FT_4222_SPIMaster_SingleWrite)(FT_HANDLE ftHandle, uint8* buffer,
                                                        uint16 bytesToWrite, uint16* sizeTransferred,
                                                        BOOL isEndTransaction);
typedef FT4222_STATUS (__stdcall FT_4222_SPIMaster_SingleReadWrite)(FT_HANDLE ftHandle, uint8* readBuffer,
                                                        uint8* writeBuffer, uint16 sizeToTransfer,
                                                        uint16* sizeTransferred, BOOL isEndTransaction);
typedef FT4222_STATUS (__stdcall FT_4222_SPISlave_Init)(FT_HANDLE pHandle);
typedef FT4222_STATUS (__stdcall FT_4222_SPISlave_InitEx)(FT_HANDLE pHandle, SPI_SlaveProtocol protocolOpt);
typedef FT4222_STATUS (__stdcall FT_4222_SPISlave_GetRxStatus)(FT_HANDLE pHandle, uint16_t* pRxSize);
typedef FT4222_STATUS (__stdcall FT_4222_SPISlave_Read)(FT_HANDLE ftHandle, uint8* buffer,
                                                        uint16 bytesToRead, uint16* sizeOfRead);
typedef FT4222_STATUS (__stdcall FT_4222_SPISlave_Write)(FT_HANDLE ftHandle, uint8* buffer,
                                                         uint16 bytesToWrite, uint16* sizeTransferred);
typedef FT4222_STATUS (__stdcall FT_4222_UnInitialize)(FT_HANDLE ftHandle);
typedef FT4222_STATUS (__stdcall FT_4222_SPI_Reset)(FT_HANDLE ftHandle);
typedef FT4222_STATUS (__stdcall FT_4222_SPI_SetDrivingStrength)(FT_HANDLE ftHandle,
                                                                 SPI_DrivingStrength clkStrength,
                                                                 SPI_DrivingStrength ioStrength,
                                                                 SPI_DrivingStrength ssoStrength);

//function pointer into the libFT4222 dll
extern FT_4222_UnInitialize         *ft4222_unInitialize;
extern FT_4222_SetClock             *ft4222_setClock;
extern FT_4222_GetClock             *ft4222_getClock;
extern FT_4222_GetVersion           *ft4222_getVersion;
extern FT_4222_GetMaxTransferSize   *ft4222_GetMaxTransferSize;
extern FT_4222_GPIO_Init            *ft4222_GPIO_Init;
extern FT_4222_GPIO_Read            *ft4222_GPIO_Read;
extern FT_4222_GPIO_Write           *ft4222_GPIO_Write;

extern FT_4222_SPIMaster_Init               *ft4222_SPIMaster_Init;
extern FT_4222_SPIMaster_SingleRead         *ft4222_SPIMaster_SingleRead;
extern FT_4222_SPIMaster_SingleWrite        *ft4222_SPIMaster_SingleWrite;
extern FT_4222_SPIMaster_SingleReadWrite    *ft4222_SPIMaster_SingleReadWrite;
extern FT_4222_SPISlave_Init                *ft4222_SPISlave_Init;
extern FT_4222_SPISlave_InitEx              *ft4222_SPISlave_InitEx;
extern FT_4222_SPISlave_GetRxStatus         *ft4222_SPISlave_GetRxStatus;
extern FT_4222_SPISlave_Read                *ft4222_SPISlave_Read;
extern FT_4222_SPISlave_Write               *ft4222_SPISlave_Write;
extern FT_4222_UnInitialize                 *ft4222_UnInitialize;
extern FT_4222_SPI_Reset                    *ft4222_SPI_Reset;
extern FT_4222_SPI_SetDrivingStrength       *ft4222_SPI_SetDrivingStrength;

// functions
bool FT4222H_loadLibraries();
void FT4222H_unloadLibraries();

#endif // FT222H_H
