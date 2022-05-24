/*
 * FT222H.cpp
 *
 *  Created on: 13.03.2016
 *      Author: ed / jp
 */

#include "libTMCComm/spi/FT4222H/FT4222H.h"
#include <QDebug>
#include "Debug.h"
#include "BasicTools.h"

// handle for the ftd2xx library
static HMODULE libHandle_ftd2xx = nullptr;

// handle for the LibFT4222 library
static HMODULE libHandle_LibFT4222 = nullptr;

// function pointer into ftd2xx.dll
FT_CreateDeviceInfoList             *ftd2xx_createDeviceInfoList;
FT_GetDeviceInfoList                *ftd2xx_getDeviceInfoList;
FT_GetDeviceInfoDetail              *ftd2xx_getDeviceInfoDetail;
FT_OpenEx                           *ftd2xx_openEx;
FT_Close                            *ftd2xx_close;
FT_SetTimeouts                      *ftd2xx_setTimeouts;
FT_Read                             *ftd2xx_read;
FT_Write                            *ftd2xx_write;
FT_GetQueueStatus                   *ftd2xx_getQueueStatus;
FT_GetStatus                        *ftd2xx_getStatus;
FT_GetDriverVersion                 *ftd2xx_getDriverVersion;
FT_GetLibraryVersion                *ftd2xx_getLibraryVersion;

// function pointer into LibFT4222.dll
FT_4222_UnInitialize                *ft4222_unInitialize;
FT_4222_SetClock                    *ft4222_setClock;
FT_4222_GetClock                    *ft4222_getClock;
FT_4222_GetVersion                  *ft4222_getVersion;
FT_4222_GetMaxTransferSize          *ft4222_GetMaxTransferSize;
FT_4222_GPIO_Init                   *ft4222_GPIO_Init;
FT_4222_GPIO_Read                   *ft4222_GPIO_Read;
FT_4222_GPIO_Write                  *ft4222_GPIO_Write;

FT_4222_SPIMaster_Init              *ft4222_SPIMaster_Init;
FT_4222_SPISlave_InitEx             *ft4222_SPISlave_InitEx;
FT_4222_SPIMaster_SingleRead        *ft4222_SPIMaster_SingleRead;
FT_4222_SPIMaster_SingleWrite       *ft4222_SPIMaster_SingleWrite;
FT_4222_SPIMaster_SingleReadWrite   *ft4222_SPIMaster_SingleReadWrite;
FT_4222_SPISlave_Init               *ft4222_SPISlave_Init;
FT_4222_SPISlave_GetRxStatus        *ft4222_SPISlave_GetRxStatus;
FT_4222_SPISlave_Read               *ft4222_SPISlave_Read;
FT_4222_SPISlave_Write              *ft4222_SPISlave_Write;
FT_4222_UnInitialize                *ft4222_UnInitialize;
FT_4222_SPI_Reset                   *ft4222_SPI_Reset;
FT_4222_SPI_SetDrivingStrength      *ft4222_SPI_SetDrivingStrength;

bool ftd2xx_loadLibrary(void)
{
    // If the library has already been loaded, return early
    if (libHandle_ftd2xx != nullptr)
        return true;

    // Load the library
    libHandle_ftd2xx = LoadLibrary(L"ftd2xx.dll");
    if (libHandle_ftd2xx == nullptr)
    {
        qDebug() << __METHOD_NAME__ << "failed to load ftd2xx.dll";
        return false;
    }

    // Load all needed API functions
    ftd2xx_createDeviceInfoList = reinterpret_cast<FT_CreateDeviceInfoList *> (GetProcAddress(libHandle_ftd2xx, "FT_CreateDeviceInfoList"));
    ftd2xx_getDeviceInfoList    = reinterpret_cast<FT_GetDeviceInfoList *>    (GetProcAddress(libHandle_ftd2xx, "FT_GetDeviceInfoList"));
    ftd2xx_getDeviceInfoDetail  = reinterpret_cast<FT_GetDeviceInfoDetail *>  (GetProcAddress(libHandle_ftd2xx, "FT_GetDeviceInfoDetail"));
    ftd2xx_openEx               = reinterpret_cast<FT_OpenEx *>               (GetProcAddress(libHandle_ftd2xx, "FT_OpenEx"));
    ftd2xx_close                = reinterpret_cast<FT_Close *>                (GetProcAddress(libHandle_ftd2xx, "FT_Close"));
    ftd2xx_setTimeouts          = reinterpret_cast<FT_SetTimeouts *>          (GetProcAddress(libHandle_ftd2xx, "FT_SetTimeouts"));
    ftd2xx_read                 = reinterpret_cast<FT_Read *>                 (GetProcAddress(libHandle_ftd2xx, "FT_Read"));
    ftd2xx_write                = reinterpret_cast<FT_Write *>                (GetProcAddress(libHandle_ftd2xx, "FT_Write"));
    ftd2xx_getQueueStatus       = reinterpret_cast<FT_GetQueueStatus *>       (GetProcAddress(libHandle_ftd2xx, "FT_GetQueueStatus"));
    ftd2xx_getStatus            = reinterpret_cast<FT_GetStatus *>            (GetProcAddress(libHandle_ftd2xx, "FT_GetStatus"));
    ftd2xx_getDriverVersion     = reinterpret_cast<FT_GetDriverVersion *>     (GetProcAddress(libHandle_ftd2xx, "FT_GetDriverVersion"));
    ftd2xx_getLibraryVersion    = reinterpret_cast<FT_GetLibraryVersion *>    (GetProcAddress(libHandle_ftd2xx, "FT_GetLibraryVersion"));

    // check correct load of API functions
    if (ftd2xx_createDeviceInfoList && ftd2xx_getDeviceInfoList
        && ftd2xx_getDeviceInfoDetail && ftd2xx_openEx && ftd2xx_close && ftd2xx_setTimeouts
        && ftd2xx_read && ftd2xx_write && ftd2xx_getQueueStatus && ftd2xx_getStatus
        && ftd2xx_getDriverVersion && ftd2xx_getLibraryVersion)
    {
        return true;
    }
    else
    {
        qDebug() << __METHOD_NAME__ << "Failed to load library functions.";

        // Unload the library again to prevent subsequent calls to this functions to falsely report successful loading
        FreeLibrary(libHandle_ftd2xx);
        libHandle_ftd2xx = nullptr;

        return false;
    }
}

bool LibFT4222_loadLibrary(void)
{
    // If the library has already been loaded, return early
    if (libHandle_LibFT4222 != nullptr)
        return true;

    // Load the library
    libHandle_LibFT4222 = LoadLibrary(L"LibFT4222.dll");        // located at: C:\Windows\SysWOW64\LibFT4222.dll
    if (libHandle_LibFT4222 == nullptr)
    {
        qDebug() << __METHOD_NAME__ << "failed to load LibFT4222.dll";
        return false;
    }

    // Load all needed API functions
    ft4222_unInitialize              = reinterpret_cast<FT_4222_UnInitialize *>              (GetProcAddress(libHandle_LibFT4222, "FT4222_UnInitialize"));
    ft4222_setClock                  = reinterpret_cast<FT_4222_SetClock *>                  (GetProcAddress(libHandle_LibFT4222, "FT4222_SetClock"));
    ft4222_getClock                  = reinterpret_cast<FT_4222_GetClock *>                  (GetProcAddress(libHandle_LibFT4222, "FT4222_GetClock"));
    ft4222_getVersion                = reinterpret_cast<FT_4222_GetVersion *>                (GetProcAddress(libHandle_LibFT4222, "FT4222_GetVersion"));
    ft4222_GetMaxTransferSize        = reinterpret_cast<FT_4222_GetMaxTransferSize *>        (GetProcAddress(libHandle_LibFT4222, "FT4222_GetMaxTransferSize"));
    ft4222_GPIO_Init                 = reinterpret_cast<FT_4222_GPIO_Init *>                 (GetProcAddress(libHandle_LibFT4222, "FT4222_GPIO_Init"));
    ft4222_GPIO_Read                 = reinterpret_cast<FT_4222_GPIO_Read *>                 (GetProcAddress(libHandle_LibFT4222, "FT4222_GPIO_Read"));
    ft4222_GPIO_Write                = reinterpret_cast<FT_4222_GPIO_Write *>                (GetProcAddress(libHandle_LibFT4222, "FT4222_GPIO_Write"));
    ft4222_SPIMaster_Init            = reinterpret_cast<FT_4222_SPIMaster_Init *>            (GetProcAddress(libHandle_LibFT4222, "FT4222_SPIMaster_Init"));
    ft4222_SPIMaster_SingleRead      = reinterpret_cast<FT_4222_SPIMaster_SingleRead *>      (GetProcAddress(libHandle_LibFT4222, "FT4222_SPIMaster_SingleRead"));
    ft4222_SPIMaster_SingleWrite     = reinterpret_cast<FT_4222_SPIMaster_SingleWrite *>     (GetProcAddress(libHandle_LibFT4222, "FT4222_SPIMaster_SingleWrite"));
    ft4222_SPIMaster_SingleReadWrite = reinterpret_cast<FT_4222_SPIMaster_SingleReadWrite *> (GetProcAddress(libHandle_LibFT4222, "FT4222_SPIMaster_SingleReadWrite"));
    ft4222_SPISlave_Init             = reinterpret_cast<FT_4222_SPISlave_Init *>             (GetProcAddress(libHandle_LibFT4222, "FT4222_SPISlave_Init"));
    ft4222_SPISlave_InitEx           = reinterpret_cast<FT_4222_SPISlave_InitEx *>           (GetProcAddress(libHandle_LibFT4222, "FT4222_SPISlave_InitEx"));
    ft4222_SPISlave_GetRxStatus      = reinterpret_cast<FT_4222_SPISlave_GetRxStatus *>      (GetProcAddress(libHandle_LibFT4222, "FT4222_SPISlave_GetRxStatus"));
    ft4222_SPISlave_Read             = reinterpret_cast<FT_4222_SPISlave_Read *>             (GetProcAddress(libHandle_LibFT4222, "FT4222_SPISlave_Read"));
    ft4222_SPISlave_Write            = reinterpret_cast<FT_4222_SPISlave_Write *>            (GetProcAddress(libHandle_LibFT4222, "FT4222_SPISlave_Write"));
    ft4222_UnInitialize              = reinterpret_cast<FT_4222_UnInitialize *>              (GetProcAddress(libHandle_LibFT4222, "FT4222_UnInitialize"));
    ft4222_SPI_Reset                 = reinterpret_cast<FT_4222_SPI_Reset *>                 (GetProcAddress(libHandle_LibFT4222, "FT4222_SPI_Reset"));
    ft4222_SPI_SetDrivingStrength    = reinterpret_cast<FT_4222_SPI_SetDrivingStrength *>    (GetProcAddress(libHandle_LibFT4222, "FT4222_SPI_SetDrivingStrength"));

    // check correct load of API functions
    if (ft4222_unInitialize && ft4222_setClock && ft4222_getClock && ft4222_getVersion && ft4222_GetMaxTransferSize
        && ft4222_SPIMaster_Init && ft4222_SPIMaster_SingleRead && ft4222_SPIMaster_SingleWrite
        && ft4222_SPIMaster_SingleReadWrite && ft4222_SPISlave_Init && ft4222_SPISlave_GetRxStatus
        && ft4222_SPISlave_Read && ft4222_SPISlave_Write && ft4222_SPISlave_InitEx && ft4222_GPIO_Init
        && ft4222_GPIO_Read && ft4222_GPIO_Write && ft4222_UnInitialize && ft4222_SPI_Reset && ft4222_SPI_SetDrivingStrength)
    {
        return true;
    }
    else
    {
        qDebug() << __METHOD_NAME__ << "Failed to load library functions.";

        // Unload the library again to prevent subsequent calls to this functions to falsely report successful loading
        FreeLibrary(libHandle_LibFT4222);
        libHandle_LibFT4222 = nullptr;

        return false;
    }
}

bool FT4222H_loadLibraries()
{
    bool loaded = true;

    if (ftd2xx_loadLibrary() == false)
    {
        qDebug() << __METHOD_NAME__ << "failed to load ftd2xx.dll!";
        qCritical() << "Failed to load ftd2xx.dll!";
        loaded = false;
    }
    else
    {
        DWORD libraryVersion;
        FT_STATUS ftStatus = ftd2xx_getLibraryVersion(&libraryVersion);
        if (ftStatus == FT_OK)
            qDebug() << __METHOD_NAME__ << "ftd2xx.dll: libraryVersion: " << BasicTools::iToHex((quint64)libraryVersion);
        else
            qDebug() << __METHOD_NAME__ << "ftd2xx.dll: libraryVersion: unknown!";

        DWORD driverVersion;
        ftStatus = ftd2xx_getDriverVersion(libHandle_ftd2xx, &driverVersion);
        if (ftStatus == FT_OK)
            qDebug() << __METHOD_NAME__ << "ftd2xx.dll: driverVersion: " << BasicTools::iToHex((quint64)driverVersion);
        else
            qDebug() << __METHOD_NAME__ << "ftd2xx.dll: driverVersion: unknown!";
    }

    if (LibFT4222_loadLibrary() == false)
    {
        qDebug() << __METHOD_NAME__ << "failed to load LibFT4222.dll!";
        qCritical() << "Failed to load LibFT4222.dll!";
        loaded = false;
    }

    return loaded;
}

void FT4222H_unloadLibraries()
{
    if ((libHandle_LibFT4222 != nullptr) && FreeLibrary(libHandle_LibFT4222))
        libHandle_LibFT4222 = nullptr;

    if ((libHandle_ftd2xx != nullptr) && FreeLibrary(libHandle_ftd2xx))
        libHandle_ftd2xx = nullptr;
}
