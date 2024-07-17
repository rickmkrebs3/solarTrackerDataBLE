// Created by Rick Krebs on 7/16/24.
//
#include <string>
#include <set>

#ifndef SOLARBT_SOLARBTADAPTER_H
#define SOLARBT_SOLARBTADAPTER_H

#endif //SOLARBT_SOLARBTADAPTER_H

class Interface1
{
public:
    Interface1() { }
    virtual ~Interface1() { }
    virtual void method1() = 0; // "= 0" part makes this method purely virtual and a class abstract
    virtual void method2() = 0;
};

class BTProfile : public Interface1
{
private:
    static const int myMember; // prev int myMember
    static const int yourMember;

public:
    BTProfile() { }
    ~BTProfile() { }
    void method1();
    void method2();
};

class Interface2
{
public:
    Interface2() { }
    virtual ~Interface2() { }
    virtual void method1() = 0; // "= 0" part makes this method purely virtual and a class abstract
    virtual void method2() = 0;
};

class Context : public Interface2
{
private:
    static const int myMember; // prev int myMember
    static const int yourMember;

public:
    Context() { }
    ~Context() { }
    void method1();
    void method2();
};

//
class BTLeAdvertiser
{
public:
private:

};
//

class BTLeScanner
{

};
//

class BTDevice
{

};
//

class BTServerSocket // inherited methods from Java: java.lang.Object, java.io.Closeable, and java.lang.AutoCloseable
{


};
//

// class that represents an immutable universally unique identifier (UUID) (128-bit value)
class UUID
{

};

class Duration // in Java implemented interfaces include: Serializable, Comparable<Duration>, and TemporalAmount
{

};
//


class solarBTAdapter
{
public:
    bool cancelDiscovery();                         // cancel the current device discovery process

    static bool checkBTAddr(std::string address);   // validate a string BT address such as "00:43:A8:23:10:F0",
                                                    // alphabetic characters must be uppercase to be valid
    void closeProfileProxyIint(int unusedProfile, BTProfile proxy); // close the profile proxy connection to the Service

    bool disable();                                 // deprecated in API level 33

    bool enable();                                  // deprecated

    std::string getAddress();                       // returns the HW address of the local BT adapter

    BTLeAdvertiser getBTLeAdvertiser();             // returns a BTLeAdvertiser object for BT LE advertising operations

    BTLeScanner getBTLeScanner();                   // returns a BTLeScanner object for BT LE scan operations

    std::set <BTDevice> getBondedDevices();         // return the set of BTDevice objects that are bonded (paired) to
                                                    // the local adapter
    static solarBTAdapter getDefaultAdapter();           // deprecated

    Duration getDiscoverableTimeout();              // get the timeout duration of the SCAN_MODE_CONNECTABLE_
                                                    // DISCOVERABLE
    int getLeMaximumAdvertisingDataLength();        // returns the maximum LE advertising data length in bytes, if LE
                                                    // Extended Advertising feature is supported, is 0 otherwise
    int getMaxConnectedAudioDevices();

    std::string getName();

    int getProfileConnectionState(int profile);

    // bool getProfileProxy(Context context, BTProfile.ServiceListener listener, int profile); // get profile proxy object

    BTDevice getRemoteDevice(std::byte address); // prev std::byte[] address);

    BTDevice getRemoteDevice(std::string address);

    BTDevice getRemoteLeDevice(std::string, int addressType);

    int getScanMode();

    int getState();

    bool isDiscovering();

    bool isEnabled();

    bool isLe2MPhySupported();

    int isLeAudioBroadcastAssistantSupported();

    int isLeAudioBroadcastSourceSupported();

    int isLeAudioSupported();

    bool isLeCodedPhySupported();

    bool isLeExtendedAdvertisingSupported();

    bool isLePeriodicAdvertisingSupported();

    bool isMultipleAdvertisementSupported();

    bool isOffloadedFilteringSupported();

    bool isOffloadedScanBatchingSupported();

    BTServerSocket listenUsingInsecureL2capChannel();

    BTServerSocket listenUsingInsecureRFcommWithServiceRecord(std::string name, UUID uuid);

    BTServerSocket listenUsingL2capChannel();

    BTServerSocket listenUsingRFcommWithServiceRecord(std::string name, UUID uuid);

    bool setName(std::string name);

    bool startDiscovery();

    // bool startLeScan(UUID serviceUuids, solarBTAdapter.LeScanCallback callback);

    // bool startLeScan(solarBTAdapter.LeScanCallback callback);    // deprecated

    // void stopLeScan(solarBTAdapter.LeScanCallback callback);

private:
    std::string ACTION_CONNECTION_STATE_CHANGED;    // used to broadcast the change in connection state of the local BT
                                                    // adapter to a remote device profile
    std::string ACTION_DISCOVERY_FINISHED;          // broadcast action: the local BT adapter has finished the device
                                                    // discovery process
    std::string ACTION_DISOVERY_STARTED;            // broadcast action: the local BT adapter has started the remote
                                                    // device discovery
    std::string ACTION_LOCAL_NAME_CHANGED;          // broadcast action: the local BT adapter has changed its friendly
                                                    // BT name
    std::string ACTION_REQUEST_DISCOVERABLE;        // activity action: show a system activity that requests
                                                    // discoverable mode
    std::string ACTION_REQUEST_ENABLE;              // activity action: show a system activity that allows the user to
                                                    // turn on BT
    std::string ACTION_SCAN_MODE_CHANGED;           // broadcast action indicates that the BT scan mode of the local
                                                    // adapter has changed
    std::string ACTION_STATE_CHANGED;               // broadcast action: the state of the local BT adapter has been
                                                    // changed
    int ERROR{};                                      // sentinel error value for this class

    std::string EXTRA_CONNECTION_STATE;             // represents the current connection state; this extra used by
                                                    // ACTION_CONNECTION_STATE_CHANGED
                                                    // std::string
    std::string EXTRA_DISCOVERABLE_DURATION;        // used as an optional int extra field in ACTION_REQUEST_DISCOVERABLE
                                                    // in order to request a specific discoverability duration in sec.
    std::string EXTRA_LOCAL_NAME;                   // used as a string extra field in ACTION_LOCAL_NAME_CHANGED to
                                                    // request the local BT name
    std::string EXTRA_PREVIOUS_CONNECTION_STATE;    // extra used by ACTION_CONNECTION_STATE_CHANGED to represent the
                                                    // previous connection state
    std::string EXTRA_PREVIOUS_SCAN_MODE;           // used as an extra int field in ACTION_SCAN_MODE_CHANGED to
                                                    // request the previous scan mode
    std::string EXTRA_PREVIOUS_STATE;               // used as an int extra field in ACTION_STATE_CHANGED to request the
                                                    // previous power state
    std::string EXTRA_SCAN_MODE;                    // used as an int extra field in ACTION_SCAN_MODE_CHANGED to request
                                                    // the current scan mode
    std::string EXTRA_STATE;                        // used as an int extra field in ACTION_STATE_CHANGED to request the
                                                    // current power state
    int SCAN_MODE_CONNECTABLE{};                      // indicates that the inquiry scan is disabled but that the page
                                                    // scan is enabled on the local BT adapter
    int SCAN_MODE_CONNECTABLE_DISCOVERABLE{};         // indicates that both the inquiry and page scan are enabled on the
                                                    // local BT adapter
    int SCAN_MODE_NONE{};                             // indicates that both the inquiry scan and page scan are disabled
                                                    // on the local BT adapter
    int STATE_CONNECTED{};                            // the profile is in the connected state

    int STATE_CONNECTING{};                           // the profile is in the connecting state

    int STATE_DISCONNECTED{};                         // the profile is in the disconnected state

    int STATE_DISCONNECTING{};                        // the profile is in the disconnecting state

    int STATE_OFF{};                                  // indicates the local BT adapter is off

    int STATE_ON{};                                   // indicates the local BT adapter is on and ready for use

    int STATE_TURNING_OFF{};                          // indicates the local BT adapter is turning off

    int STATE_TURNING_ON{};                           // indicates the local BT adapter is turning on

protected:
    void finalize();                                // called by the garbage collector on an object when garbage
                                                    // collection determines that there are no more references to the
                                                    // object (Jave only?) -- thus applicable here?
};

