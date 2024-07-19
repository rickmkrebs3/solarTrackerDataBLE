// Created by Rick Krebs on 7/16/24.
//
#include "solarBTAdapter.h"
#include <iostream>

using namespace std;

/* do not need constructor or destructor definition as already done this in header
// constructor
solarBTAdapter::solarBTAdapter() noexcept
{
    // static std::string ACTION_CONNECTION_STATE_CHANGED;
}

// destructor
solarBTAdapter::~solarBTAdapter() noexcept
{ }
*/

// cancel the current device discovery process
bool solarBTAdapter::cancelDiscovery()
{ }

// validate a string BT address such as "00:43:A8:23:10:F0", alphabetic characters must be uppercase to be valid
bool solarBTAdapter::checkBTAddr(std::string address)
{ }

// close the profile proxy connection to the Service
void solarBTAdapter::closeProfileProxyIint(int unusedProfile, BTProfile proxy)
{ }

// deprecated in API level 33
bool solarBTAdapter::disable()
{ }

// deprecated
bool solarBTAdapter::enable()
{ }

// returns the HW address of the local BT adapter
std::string solarBTAdapter::getAddress()
{ }

// returns a BTLeAdvertiser object for BT LE advertising operations
BTLeAdvertiser solarBTAdapter::getBTLeAdvertiser()
{ }

// returns a BTLeScanner object for BT LE scan operations
BTLeScanner solarBTAdapter::getBTLeScanner()
{ }

// return the set of BTDevice objects that are bonded (paired) to the local adapter
std::set <BTDevice> solarBTAdapter::getBondedDevices()
{ }

// deprecated
solarBTAdapter solarBTAdapter::getDefaultAdapter()
{ }

// get the timeout duration of the SCAN_MODE_CONNECTABLE_DISCOVERABLE
Duration solarBTAdapter::getDiscoverableTimeout()
{ }

// returns the maximum LE advertising data length in bytes, if LE Extended Advertising feature is supported, is 0 otherwise
int solarBTAdapter::getLeMaximumAdvertisingDataLength()
{ }

int solarBTAdapter::getMaxConnectedAudioDevices()
{ }

std::string solarBTAdapter::getName()
{ }

int solarBTAdapter::getProfileConnectionState(int profile)
{ }

BTDevice solarBTAdapter::getRemoteDevice(std::byte address) // prev std::byte[] address)
{ }

BTDevice solarBTAdapter::getRemoteDevice(std::string address)
{ }

BTDevice solarBTAdapter::getRemoteLeDevice(std::string, int addressType)
{ }

int solarBTAdapter::getScanMode()
{
    return 0;
}

int solarBTAdapter::getState()
{
    return 0;
}

bool solarBTAdapter::isDiscovering()
{
    return true;
}

bool solarBTAdapter::isEnabled()
{
    return true;
}

bool solarBTAdapter::isLe2MPhySupported()
{
    return true;
}

int solarBTAdapter::isLeAudioBroadcastAssistantSupported()
{
    return 0;
}

int solarBTAdapter::isLeAudioBroadcastSourceSupported()
{
    return 0;
}

int solarBTAdapter::isLeAudioSupported()
{
    return 0;
}

bool solarBTAdapter::isLeCodedPhySupported()
{
    return true;
}

bool solarBTAdapter::isLeExtendedAdvertisingSupported()
{
    return true;
}

bool solarBTAdapter::isLePeriodicAdvertisingSupported()
{
    return true;
}

bool solarBTAdapter::isMultipleAdvertisementSupported()
{
    return true;
}

bool solarBTAdapter::isOffloadedFilteringSupported()
{
    return true;
}

bool solarBTAdapter::isOffloadedScanBatchingSupported()
{
    return true;
}

BTServerSocket solarBTAdapter::listenUsingInsecureL2capChannel()
{ }

BTServerSocket solarBTAdapter::listenUsingInsecureRFcommWithServiceRecord(std::string const name, UUID uuid)
{ }

BTServerSocket solarBTAdapter::listenUsingL2capChannel()
{ }

BTServerSocket solarBTAdapter::listenUsingRFcommWithServiceRecord(std::string const name, UUID uuid)
{ }

bool solarBTAdapter::setName(std::string const name)
{ }

bool solarBTAdapter::startDiscovery()
{ }

/*
// get profile proxy object
bool solarBTAdapter::getProfileProxy(Context context, BTProfile.ServiceListene const listener, int profile)
{
    return false;
}
*/

/*
bool solarBTAdapter::startLeScan(UUID serviceUuids, solarBTAdapter.LeScanCallback callback)
{ }

// deprecated
bool solarBTAdapter::startLeScan(solarBTAdapter.LeScanCallback callback)
{ }

void solarBTAdapter::stopLeScan(solarBTAdapter.LeScanCallback callback)
{ }
*/

void BTProfile::method1()
{

}

void BTProfile::method2()
{

}