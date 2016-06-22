
//
// Copyright (C) 2006 Andras Varga and Levente M�sz�ros
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include <algorithm>
#include "DevMac.h"
#include "RadioState.h"
//#include "PhyState.h"
#include "IInterfaceTable.h"
#include "InterfaceTableAccess.h"
#include "PhyControlInfo_m.h"
#include "Ieee802Ctrl_m.h"
#include <math.h>
#include <ctime>
 // Uniform from interval [0,1)

Define_Module(DevMac);

// don't forget to keep synchronized the C++ enum and the runtime enum definition
Register_Enum(DevMac,
   (DevMac::IDLE,
    DevMac::DEFER,
    DevMac::WAITDIFS,
    DevMac::BACKOFF,
    DevMac::WAITACK,
    DevMac::WAITBROADCAST,
    DevMac::WAITCTS,
    DevMac::WAITSIFS,
    DevMac::RECEIVE));

// don't forget to keep synchronized the C++ enum and the runtime enum definition
Register_Enum(RadioState,
   (RadioState::IDLE,
    RadioState::RECV,
    RadioState::TRANSMIT,
    RadioState::SLEEP));

Register_Enum(PhyState,
   (PhyState::IDLE,
    PhyState::BUSY));

/****************************************************************
 * Construction functions.
 */
DevMac::DevMac()
{
    endBackoff = NULL;
    endTimeout = NULL;
    endReserve = NULL;
    mediumStateChange = NULL;
    pendingRadioConfigMsg = NULL;
}

DevMac::~DevMac()
{
    cancelAndDelete(endTimeout);
    cancelAndDelete(mediumStateChange);

    if (pendingRadioConfigMsg)
        delete pendingRadioConfigMsg;
}

/****************************************************************
 * Initialization functions.
 */
void DevMac::initialize(int stage)
{
    WirelessMacBase::initialize(stage);

    if (stage == 0)
    {
        EV << "Initializing stage 0\n";

        // initialize parameters
        maxQueueSize = par("maxQueueSize");
        bitrate = par("bitrate");
        basicBitrate = 600000; //FIXME make it parameter

        const char *addressString = par("address");
        if (!strcmp(addressString, "auto")) {
            // assign automatic address
            address = MACAddress::generateAutoAddress();
            // change module parameter from "auto" to concrete address
            par("address").setStringValue(address.str().c_str());
        }
        else
            address.setAddress(addressString);

        numReceived = 0;
        // subscribe for the information of the carrier sense
        nb->subscribe(this, NF_RADIOSTATE_CHANGED);

        // initalize self messages
        endTimeout = new cMessage("Timeout");
        mediumStateChange = new cMessage("MediumStateMange");
        ackTimeout = new cMessage("ack-not-received-timeout");
        delay_timer_30 = new cMessage("delay_timer_30");
        backoffTime = new cMessage("backoffTime");
        // create a new cMessage here

        // interface
        registerInterface();

        // obtain pointer to external queue
        // initializeQueueModule();

        // state variables
        fsm.setName("DevMac State Machine");
        sequenceNumber = 0;
        radioState = RadioState::IDLE;
        retryCounter = 0;
        backoff = false;

        // statistics
        numRetryNode0 = 0;
        numRetryNode1 = 0;
        numRetryNode2 = 0;
        numRetryNode3 = 0;
        numRetryNode4 = 0;

        delay=0.02;



        numSentWithoutRetry = 0;
        numGivenUp = 0;
        numCollision = 0;
        numSent = 0;
        numReceived = 0;
        numSentBroadcast = 0;
        numReceivedBroadcast = 0;
        numRetransmit = 0;
        numBits = 0;
        numBitsN1= 0;
        numBitsN2= 0;
        numBitsRB= 0;
        numAcks = 0;
        numSets = 0;
        seed = getParentModule()->getParentModule()->getIndex(); //need different seeds for nodes


        srand(time(0));
        min_back_off = 0.2;
        randvariableb = 1;

        currentpacketSequenceNumber_6 = 0;
        previouspacketSequenceNumber_6 = -1;
        currentpacketSequenceNumber_5 = 0;
        previouspacketSequenceNumber_5 = -1;
        currentpacketSequenceNumber_4 = 0;
        previouspacketSequenceNumber_4 = -1;
        currentpacketSequenceNumber_3 = 0;
        previouspacketSequenceNumber_3 = -1;
        currentpacketSequenceNumber_2 = 0;
        previouspacketSequenceNumber_2 = -1;
        currentpacketSequenceNumber = 0;
        previouspacketSequenceNumber = -1;
        iris_hardware_delay = 0;
        total_delay=0;
        begin_time=0;
        end_time=0;

        //change the ACK mode ackMode {Explicit, Implicit}
        ackMode= Explicit;
        // Here the delay to be added in the multi-hop protocol
        protocol_delay = 0;

        total_packets = 100;

        //delays for each test
        iris_hardware_delay = 0.008;
        iris_hardware_delay_ACK = 0.002;

        total_delay = 0.000 + iris_hardware_delay; //delay < timeout (no retransmissions)

        macState = DevMac::IDLE;

        stateVector.setName("State");

        endTimeout = NULL;

        stateVector.setEnum("DevMac");
        radioStateVector.setName("RadioState");
        radioStateVector.setEnum("RadioState");

        endToEndDelayVec.setName("End-to-End Delay");

        // initialize watches
        WATCH(fsm);
        WATCH(radioState);
        WATCH(phyState);
        WATCH(retryCounter);
        WATCH(backoff);
        WATCH(numRetry);
        WATCH(numSentWithoutRetry);
        WATCH(numGivenUp);
        WATCH(numCollision);
        WATCH(numSent);
        WATCH(numReceived);
        WATCH(numSentBroadcast);
        WATCH(numReceivedBroadcast);
        WATCH(numRetransmit);
        WATCH(min_back_off);
        WATCH(begin_time);
        WATCH(end_time);
        WATCH(numBits);
        WATCH(numAcks);
    }
}

void DevMac::registerInterface()
{
    IInterfaceTable *ift = InterfaceTableAccess().getIfExists();
    if (!ift)
        return;

    InterfaceEntry *e = new InterfaceEntry();

    // interface name: NetworkInterface module's name without special characters ([])
    char *interfaceName = new char[strlen(getParentModule()->getFullName()) + 1];
    char *d = interfaceName;
    for (const char *s = getParentModule()->getFullName(); *s; s++)
        if (isalnum(*s))
            *d++ = *s;
    *d = '\0';

    e->setName(interfaceName);
    delete [] interfaceName;

    // address
    e->setMACAddress(address);
    e->setInterfaceToken(address.formInterfaceIdentifier());

    // FIXME: MTU on 802.11 = ?
    e->setMtu(par("mtu"));

    // capabilities
    e->setBroadcast(true);
    e->setMulticast(true);
    e->setPointToPoint(false);

    // add
    ift->addInterface(e, this);
}



/****************************************************************
 * Message handling functions.
 */
void DevMac::handleSelfMsg(cMessage *msg)
{
	//used for retransmissions: called by the scheduledAt function ()

	if(msg == ackTimeout)
	{
    EV << "received self message: " << msg << endl;

    ev << "Total packets in the MAC buffer is : " << transmissionQueue.size() <<endl;

	sendDownPendingRadioConfigMsg();

	macState = DevMac::IDLE; // for all node

	if(macState != WAITACK){
		if(!transmissionQueue.empty()) {

			//min_back_off += (backoff_fun(0)); // all nodes will seed differently because they have different initalization seeds


			EV << "INCREASED BACKOFF TIME IN MSG TRANSMIT TO :"<<min_back_off<<endl;
			//increment number of retry
			if (!strcmp(address.str().c_str(), "0A-AA-00-00-00-01")) {
				numRetryNode0++;

			}
			else if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-02")) {
				numRetryNode1++;

			}
			else if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-03")) numRetryNode2++;
			else if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-04")) numRetryNode3++;
			else if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-05")) numRetryNode4++;

			macState = DevMac::WAITACK;
			cancelEvent(backoffTime); //cancel timer before scheduling
			scheduleAt(simTime() + backoff_fun(0), backoffTime);
			//sendDataFrame(transmissionQueue.front(), true);


		}
		else
			ev<<"Empty!\n";
	}
	}
	else if(msg == delay_timer_30)
		{
		ev<<"Timer was delayed by 30 milliseconds"<<endl;
		if(!transmissionQueue.empty())
		{
			sendDataFrame(transmissionQueue.front(), true);
			cancelEvent(ackTimeout);
			macState = DevMac::WAITACK;
			scheduleAt(simTime() + min_back_off, ackTimeout);
		}
		}

	else if(msg == backoffTime)
		{
		ev<<"Backoff has re-schedule the Transmission "<<endl;
		if(!transmissionQueue.empty())
			{
			sendDataFrame(transmissionQueue.front(), true);
			cancelEvent(ackTimeout);
			macState = DevMac::WAITACK;
			scheduleAt(simTime() + min_back_off, ackTimeout);
			}
		}
		//sendDataFrame((Ieee80211DataOrMgmtFrame *)transmissionQueue.front(), false);
		//sendDownDelayed(setBasicBitrate(buildDataFrame(frameToSend)), result); // add hardware delay here


}

float DevMac::backoff_fun(int seed)
{

	//float temporary = ((float)(rand()%10))/10.0; //rand(): integer uniform rv 0-RAND_MAX
	float temporary = ((float)(rand()%10))*5.0/100.0;
	EV <<"We have chosen :"<< temporary <<" as the random backoff time"<<endl;

	return temporary;

}

void DevMac::handleUpperMsg(cPacket *msg)
{

	macState = DevMac::IDLE;
	// check for queue overflow
    if (maxQueueSize && ((int)transmissionQueue.size() == maxQueueSize))
    {
        EV << "message " << msg << " received from higher layer but MAC queue is full, dropping message\n";
        delete msg;
        return;
    }

    Ieee80211DataFrame *frame = new Ieee80211DataFrame(msg->getName());

    // copy receiver address from the control info (sender address will be set in MAC)
    Ieee802Ctrl *ctrl = check_and_cast<Ieee802Ctrl *>(msg->removeControlInfo());
    frame->setReceiverAddress(ctrl->getDest());
    delete ctrl;

    ev << "Frame bit length before encaps "<< frame->getBitLength()<<endl;

    frame->encapsulate(msg);

    EV << "frame " << frame << " received from higher layer, receiver = " << frame->getReceiverAddress() << endl;

    ASSERT(!frame->getReceiverAddress().isUnspecified());

    // fill in missing fields (receiver address, seq number), and insert into the queue
    frame->setTransmitterAddress(address);
    frame->setSequenceNumber(sequenceNumber);
    sequenceNumber = (sequenceNumber+1) % 4096;  //XXX seqNum must be checked upon reception of frames!

    min_back_off = 0.2; //initial ARQ timeout

    ev<<"The sequence number of the frame is : " <<frame->getSequenceNumber()<<endl;

   	transmissionQueue.push_back(frame); //place frame in the queue

    if(frame->getSequenceNumber() > 0){
	    if(!strcmp(address.str().c_str(),"0A-AA-00-00-00-01"))macState = DevMac::WAITACK;
		ev << "There is at least one packet waiting in the buffer "<< endl;
    }

    if (macState == DevMac::WAITACK)
    	ev << "MAC is in waitack state, queue packet\n";
    else
    	ev << "MAC is in the idle state\n";

	ev << "Packets in queue : " << transmissionQueue.size() ;

    if(macState != DevMac::WAITACK){
    	//radio and mac are ready
    	if(!transmissionQueue.empty()){

    		previouspacketSequenceNumber_5 = currentpacketSequenceNumber_5;

			if(!strcmp(address.str().c_str(),"0A-AA-00-00-00-01") ){ //OK
				sendDownPendingRadioConfigMsg();
				ev <<"Node 1: RESET BACKOFF TO :" << min_back_off << endl;
				/*
			    cancelEvent(delay_timer_30);
			    scheduleAt(simTime() + 0.02, delay_timer_30);
			    */

				sendDataFrame(transmissionQueue.front(), true);
				if(frame->getSequenceNumber() == 0){
						begin_time = simTime();
					}
				cancelEvent(ackTimeout);
				scheduleAt( simTime()+ min_back_off, ackTimeout);
				macState = DevMac::WAITACK;
			}
			else if (!strcmp(address.str().c_str(),"0A-AA-00-00-00-02")){ //OKAY
				numBitsN2+=frame->getBitLength();
				sendDownPendingRadioConfigMsg();

				ev <<"Node 2: RESET BACKOFF TO :" << min_back_off << endl;
				if(transmissionQueue.front()->getSequenceNumber() == 0 || transmissionQueue.size() == 1) {
					//check if medium is free!
					if(isMediumFree()) {
						sendDataFrame(transmissionQueue.front(), false);
						ev << "Sequence number is 0 or size of transmission buffer is 1"<<endl;
						cancelEvent(ackTimeout);
						scheduleAt( simTime()+ min_back_off, ackTimeout);
					}
					else {
						ev <<"MEDIUM IS NOT FREE!!, BACKOFF AND TRY AT AL LATER TIME"<<endl;
						cancelEvent(ackTimeout);
						scheduleAt( simTime()+ min_back_off, ackTimeout);
					}
				}
				macState = DevMac::WAITACK;

			}
			else if (!strcmp(address.str().c_str(),"0A-AA-00-00-00-03")){ //OKAY
				numBitsN2+=frame->getBitLength();
				sendDownPendingRadioConfigMsg();

				ev <<"Node 3: RESET BACKOFF TO :" << min_back_off << endl;
				sendDataFrame(transmissionQueue.front(), false);
				macState = DevMac::WAITACK;
				cancelEvent(ackTimeout);
				scheduleAt( simTime()+ min_back_off, ackTimeout);
			}
			else if (!strcmp(address.str().c_str(),"0A-AA-00-00-00-04")){ //OKAY
				numBitsN2+=frame->getBitLength();
				sendDownPendingRadioConfigMsg();

				ev <<"Node 4: RESET BACKOFF TO :" << min_back_off << endl;
				sendDataFrame(transmissionQueue.front(), false);
				macState = DevMac::WAITACK;
			}
			else if (!strcmp(address.str().c_str(),"0A-AA-00-00-00-05")){ //OKAY
				numBitsN2+=frame->getBitLength();
				sendDownPendingRadioConfigMsg();

				ev <<"Node 5: RESET BACKOFF TO :" << min_back_off << endl;
				sendDataFrame(transmissionQueue.front(), false);
				macState = DevMac::WAITACK;
			}
			else if (!strcmp(address.str().c_str(),"0A-AA-00-00-00-06")){ //OKAY
				numBitsN2+=frame->getBitLength();
				sendDownPendingRadioConfigMsg();

				ev <<"Node 6: RESET BACKOFF TO :" << min_back_off << endl;
				sendDataFrame(transmissionQueue.front(), false);
				macState = DevMac::WAITACK;
			}

			//scheduleAt( simTime()+ min_back_off, ackTimeout);
    	}
    	else {
    		//does it ever go aqui ?
    		scheduleAt(simTime() + min_back_off, ackTimeout);
    	}
    }


}
void DevMac::handleCommand(cMessage *msg)
{

}
void DevMac::handleLowerMsg(cPacket *msg)
{
	if(msg == ackTimeout)
	EV << "received message from lower layer: " << msg << endl;

    Ieee80211Frame *frame = dynamic_cast<Ieee80211Frame *>(msg); //get message
    Ieee80211DataOrMgmtFrame *ackframe = dynamic_cast<Ieee80211DataOrMgmtFrame *>(msg); //get message
    Ieee80211ACKFrame *ackframe0 = dynamic_cast<Ieee80211ACKFrame *>(msg); //get message

    if (!frame) //if not a message, the error message to the screen
        error("message from physical layer (%s)%s is not a subclass of Ieee80211Frame",
              msg->getClassName(), msg->getName());
    EV << "Self address: " << address.str().c_str() //this is the nodes address
       << ", receiver address: " << frame->getReceiverAddress() //this is the receiver address (destination address)
       << ", received frame is for us: " << isForUs(frame) << endl; //check if the data is for us !!
    int frameType = frame ? frame->getType() : -1; //this returns either a 0 or 1, letting the user to know if it is

	if(ackMode == Explicit)
	{
		if(isForUs(frame))
		{

			if (frameType == ST_DATA) { //is for us and data
				ev << "RECEIVED DATA for us\n"; //this is at the relay node and last node node
				Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame *> (frame); //blah, blah ....

				currentpacketSequenceNumber=dframe->getSequenceNumber() + 1; //same ol

				EV <<"Current sequence number is :" <<dframe->getSequenceNumber()<<endl;

				if( currentpacketSequenceNumber > previouspacketSequenceNumber) //data_number is great than the previous value
				{
					if(!(strcmp(address.str().c_str(),"0A-AA-00-00-00-03"))){
						numBits+=frame->getBitLength(); //get the length of the packet in bits
					}
					cPacket *payload = dframe->decapsulate();

					ev << "This is my address: " << address.str().c_str() <<endl; //dislay address of node

					Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

					ev << "The sequence number of the DATA packet is :" << dframe->getSequenceNumber()<<endl;

					ackframe->setSequenceNumber(dframe->getSequenceNumber());

					ev << "The sequence number of the ACK packet is :" << ackframe->getSequenceNumber()<<endl;

					sendACKFrame(ackframe); //send ack with sequence number

					previouspacketSequenceNumber = currentpacketSequenceNumber; //save sequence number as usual

					numReceived++; //increment the number of data packets received

					if(!strcmp(address.str().c_str() ,"0A-AA-00-00-00-02"))sendUpDelayed(payload,0);

					else if(!strcmp(address.str().c_str() ,"0A-AA-00-00-00-03"))sendUpDelayed(payload, 0);
					else if(!strcmp(address.str().c_str() ,"0A-AA-00-00-00-04"))sendUpDelayed(payload, 0);
					else if(!strcmp(address.str().c_str() ,"0A-AA-00-00-00-05"))sendUpDelayed(payload, 0);
					else if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-06")) {sendUpDelayed(payload, 0);}
					EV <<"Current sequence number is :" <<dframe->getSequenceNumber()<<endl;

				}
				else
				{ //if it is old, then do nothing to the data. we can
					if(!strcmp(address.str().c_str() ,"0A-AA-00-00-00-02") || !strcmp(address.str().c_str() ,"0A-AA-00-00-00-03") || !strcmp(address.str().c_str() ,"0A-AA-00-00-00-04") || !strcmp(address.str().c_str() ,"0A-AA-00-00-00-05")|| !strcmp(address.str().c_str() ,"0A-AA-00-00-00-06") ){
						Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);
						ackframe->setSequenceNumber(dframe->getSequenceNumber());
						sendACKFrame(ackframe);
					delete frame;
					}
				}
				 // number of packets received in this layer
			}
			else if (frameType == ST_ACK ) { //if this is an ack packet ..

				macState = DevMac::IDLE;

				ev << "RECEIVED ACK for us!!\n"; // trigger another transmission after some time? macState = DevMac::WAITMAC
				ev << "RESET THE BACKOFF TIMER"<<endl;
				if(!transmissionQueue.empty()){

					cancelEvent(ackTimeout); //stop timercancelEvent(ackTimeout); //stop timer

					ev << "The sequence number for the ACK is" << ackframe0->getSequenceNumber();

					if(transmissionQueue.front()->getSequenceNumber() == ackframe0->getSequenceNumber() )
					{
						transmissionQueue.pop_front(); //pop fix
						ev << "DATA HAS BEEN REMOVED-POP!!" <<endl;
						numAcks++; //...
						if((numAcks%499 == 0) && (numAcks !=0) && !(strcmp(address.str().c_str(),"0A-AA-00-00-00-02")) ){ //change here
							end_time = simTime();
							//would be nice to store previous_time_in_vector - endtime in the following record statement
							endToEndDelayVec.record(end_time);
							EV << "The last ACK was received at: " <<end_time<<endl;
							numSets++;
						}
					}
					else
						ev << "Sequence numbers did not match. DATA WAS NOT REMOVED !" <<endl;

					min_back_off = 0.2; //reset the minimum backoff

					ev << "Buffer size: " << transmissionQueue.size() <<endl;

					if((strcmp(address.str().c_str(),"0A-AA-00-00-00-01")))
					{
						if(!transmissionQueue.empty())
						{
							sendDataFrame(transmissionQueue.front(), true);
							cancelEvent(ackTimeout);
							macState = DevMac::WAITACK;
							scheduleAt( simTime()+ min_back_off, ackTimeout);
						}
					 // 3/12
					}
					else
					{
						cancelEvent(delay_timer_30);
						scheduleAt(simTime()+ protocol_delay , delay_timer_30);	// Here we add delay on the protocol

					}
					}

				else{
					cancelEvent(ackTimeout); //stop timercancelEvent(ackTimeout); //stop timer
					EV << "Cancelled event !!" <<endl;
				}
			}
			else {
				ev << "RECEIVED something other than data or ack at DevMac\n";
				delete frame;
				delete msg;
			}
		}
		else {
			ev << "Packet not for us, drop it\n";
			delete frame;
		}
	}
	else if(ackMode == Implicit)
	{
		if(isForUs(frame)) //check if the frame is for us
		{
			if (frameType == ST_DATA) { //if it is for us and is a data then continue
				ev << "RECEIVED DATA for us\n"; //this is at the relay node and last node node
				Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame*> (frame); //create a frame

				if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-06")) { //middle gets data successfully and then sends implicit ACK (Data+ack)
					currentpacketSequenceNumber_6=dframe->getSequenceNumber();
					EV << "CurrentSequenceNo:"<<currentpacketSequenceNumber_6<< "/PreviousSequenceNo:" << previouspacketSequenceNumber_6;
					if( currentpacketSequenceNumber_6 > previouspacketSequenceNumber_6){
						//if we are the second node, the relay node, then ...
						numBitsRB+=frame->getBitLength(); //get the length of the packet in bits
						cPacket *payload = dframe->decapsulate(); //get the payload , might need to be cleaned up further
						previouspacketSequenceNumber_6 = currentpacketSequenceNumber_6;
						/*Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);
						ackframe->setSequenceNumber(dframe->getSequenceNumber());
						sendACKFrame(ackframe);
						*/
						ev << "Received a data packet. Now, the PreviousSeqNumber is:" << previouspacketSequenceNumber_6 ;
						numReceived++;
						delete dframe;
						sendUpDelayed(payload, 0); //send the data packet up to the network layer
					}
					else
					{
						//if it is old, then send another ACK
						ev << "This is an old frame, sending another acknowledgment !!" <<endl;

						Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

						ackframe->setSequenceNumber(dframe->getSequenceNumber());

						sendACKFrame(ackframe);

						delete ackframe;

					}
				}

				else if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-05")) { //middle gets data successfully and then sends implicit ACK (Data+ack)

					currentpacketSequenceNumber_5=dframe->getSequenceNumber();

					EV << "CurrentSequenceNo:"<<currentpacketSequenceNumber_5<< "/PreviousSequenceNo:" << previouspacketSequenceNumber_5;

					if( currentpacketSequenceNumber_5 > previouspacketSequenceNumber_5){

						cPacket *payload = dframe->decapsulate(); //get the payload , might need to be cleaned up further

						previouspacketSequenceNumber_5 = currentpacketSequenceNumber_5;

						/*Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

						ackframe->setSequenceNumber(dframe->getSequenceNumber());

						sendACKFrame(ackframe);*/

						ev << "previous seq" << previouspacketSequenceNumber_5 ;

						numReceived++;

						delete dframe;

						sendUpDelayed(payload, 0); //send the data packet up to the network layer
					}
					else
					{ //if it is old, then do nothing to the data. we can
						ev << "This is an old frame, sending another acknowledgment !!" <<endl;
							//if we are the second nod<endl;
						Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

						ackframe->setSequenceNumber(dframe->getSequenceNumber());

						sendACKFrame(ackframe);

						delete ackframe;
					}
				}
				else if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-04")) { //middle gets data successfully and then sends implicit ACK (Data+ack)

					currentpacketSequenceNumber_4=dframe->getSequenceNumber();

					EV << "CurrentSequenceNo:"<<currentpacketSequenceNumber_4<< "/PreviousSequenceNo:" << previouspacketSequenceNumber_4;

					if( currentpacketSequenceNumber_4 > previouspacketSequenceNumber_4){

						//if we are the second node, the relay node, then ...
						numBitsRB+=frame->getBitLength(); //get the length of the packet in bits

						cPacket *payload = dframe->decapsulate(); //get the payload , might need to be cleaned up further

						Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

						ackframe->setSequenceNumber(dframe->getSequenceNumber());

						sendACKFrame(ackframe);

						previouspacketSequenceNumber_4 = currentpacketSequenceNumber_4;

						ev << "previous seq" << previouspacketSequenceNumber_4 ;

						numReceived++;

						delete dframe;

						sendUpDelayed(payload, 0); //send the data packet up to the network layer
					}
					else
					{ //if it is old, then do nothing to the data. we can
						ev << "This is an old frame, sending another acknowledgment !!" <<endl;
							//if we are the second nod<endl;
						Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

						ackframe->setSequenceNumber(dframe->getSequenceNumber());

						sendACKFrame(ackframe);

						delete ackframe;

					}
				}
				else if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-03")) { //middle gets data successfully and then sends implicit ACK (Data+ack)

					currentpacketSequenceNumber_3=dframe->getSequenceNumber();

					EV << "CurrentSequenceNo:"<<currentpacketSequenceNumber_3<< "/PreviousSequenceNo:" << previouspacketSequenceNumber_3;

					if( currentpacketSequenceNumber_3 > previouspacketSequenceNumber_3){

						/*
						//if we are the second node, the relay node, then ...
						cPacket *payload = dframe->decapsulate(); //get the payload , might need to be cleaned up further

						//Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

						//ackframe->setSequenceNumber(dframe->getSequenceNumber());

						//sendACKFrame(ackframe);

						previouspacketSequenceNumber_3 = currentpacketSequenceNumber_3;

						ev << "previous seq" << previouspacketSequenceNumber_3 ;

						numReceived++;

						delete dframe;

						sendUpDelayed(payload, 0); //send the data packet up to the network layer
						*/
						//if we are the second node, the relay node, then ...

						numBitsRB+=frame->getBitLength(); //get the length of the packet in bits

						cPacket *payload = dframe->decapsulate(); //get the payload , might need to be cleaned up further

						Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

						ackframe->setSequenceNumber(dframe->getSequenceNumber());

						sendACKFrame(ackframe);

						previouspacketSequenceNumber_3 = currentpacketSequenceNumber_3;

						ev << "previous seq" << previouspacketSequenceNumber_3 ;

						numReceived++;

						delete dframe;

						sendUpDelayed(payload, 0); //send the data packet up to the network layer
					}
					else
					{ //if it is old, then do nothing to the data. we can
						ev << "This is an old frame, sending another acknowledgment !!" <<endl;
							//if we are the second nod<endl;
						Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

						ackframe->setSequenceNumber(dframe->getSequenceNumber());

						sendACKFrame(ackframe);

						delete ackframe;

					}
				}
				else if (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-02")) { //middle gets data successfully and then sends implicit ACK (Data+ack)

					currentpacketSequenceNumber_2=dframe->getSequenceNumber();

					EV << "CurrentSequenceNo:"<<currentpacketSequenceNumber_2<< "/PreviousSequenceNo:" << previouspacketSequenceNumber_2;

					if( currentpacketSequenceNumber_2 > previouspacketSequenceNumber_2){
						//if we are the second node, the relay node, then ...

						cPacket *payload = dframe->decapsulate(); //get the payload , might need to be cleaned up further

						previouspacketSequenceNumber_2 = currentpacketSequenceNumber_2;

						ev << "previous seq" << previouspacketSequenceNumber_2 ;

						numReceived++;

						delete dframe;

						sendUpDelayed(payload, 0); //send the data packet up to the network layer
							//and let it check if it is the right data for the node, if it isn't then network will relay the data packet.
							//the network layer knows its ip address. the impliciation is hiden from this layer and has been implemented correctly.
					}
					else
					{ //if it is old, then do nothing to the data. we can
						ev << "This is an old frame, sending another acknowledgment !!" <<endl;

						Ieee80211DataOrMgmtFrame* ackframe = check_and_cast<Ieee80211DataOrMgmtFrame*> (frame);

						ackframe->setSequenceNumber(dframe->getSequenceNumber());

						sendACKFrame(ackframe);

						delete ackframe;

					}
				}
				 // number of packets received in this layer
			}
			else if (frameType == ST_ACK ) { //check if the data received is an ack packet, the second node cares about acks,
				//not the first node, in implicit mode
				ev << "RECEIVED ACK for us!!\n"; // trigger another transmission after some time? macState = DevMac::WAITMAC

				if(!transmissionQueue.empty()){
					 EV <<"The current sequence number in the buffer is :"<<transmissionQueue.front()->getSequenceNumber()<<"/ The received or ACK packet sequence number is :" <<ackframe0->getSequenceNumber() <<endl;
					if(transmissionQueue.front()->getSequenceNumber() ==  ackframe0->getSequenceNumber())
					{
						cancelEvent(ackTimeout); //stop timercancelEvent(ackTimeout); //stop timer
						ev << "DATA with sequence number"<<transmissionQueue.front()->getSequenceNumber()<<"will has been removed-POPPED!!" <<endl;
						transmissionQueue.pop_front(); //pop fix
						numAcks++; //...
						if((numAcks%499 == 0) && (numAcks !=0) && !(strcmp(address.str().c_str(),"0A-AA-00-00-00-02")) ){ //change here
							end_time = simTime();
							//would be nice to store previous_time_in_vector - endtime in the following record statement
							endToEndDelayVec.record(end_time);
							numSets++;
							EV << "The last ACK was received at: " <<end_time<<endl;

						}
						ev << "numAcks is :" <<numAcks <<endl;
						ev << "RESET THE BACKOFF TIMER"<<endl;
						min_back_off = 0.2; //reset the minimum back off
						if(!transmissionQueue.empty()) {
							if((!strcmp(address.str().c_str() ,"0A-AA-00-00-00-01")))
							{
								previouspacketSequenceNumber = currentpacketSequenceNumber; //record //increase the sequence number, since the first one is 0,
								sendDataFrame(transmissionQueue.front(), true);
							}
							else if((!strcmp(address.str().c_str() ,"0A-AA-00-00-00-02")))
							{
								previouspacketSequenceNumber_2 = currentpacketSequenceNumber_2; //record //increase the sequence number, since the first one is 0,
								sendDataFrame(transmissionQueue.front(), true);
							}
							else if(!strcmp(address.str().c_str() ,"0A-AA-00-00-00-03"))
							{
								previouspacketSequenceNumber_3 = currentpacketSequenceNumber_3;  //increase the sequence number, since the first one is 0,
								sendDataFrame(transmissionQueue.front(), true);
							}
							else if(!strcmp(address.str().c_str() ,"0A-AA-00-00-00-04"))
							{
								currentpacketSequenceNumber_4=currentpacketSequenceNumber_4; //increase the sequence number, since the first one is 0,
								sendDataFrame(transmissionQueue.front(), true);
							}
							else if((!strcmp(address.str().c_str() ,"0A-AA-00-00-00-05")))
							{
								currentpacketSequenceNumber_5=currentpacketSequenceNumber_5;; //increase the sequence number, since the first one is 0,
								sendDataFrame(transmissionQueue.front(), true);
							}
							scheduleAt(simTime() + min_back_off, ackTimeout);

						} // 3/12
					}
					else
						ev << "Sequence numbers did not match. DATA WAS NOT REMOVED !" <<endl;
				}
			}
			else { //we got shit from the below !! cannot be recognized, then delete!
				ev << "RECEIVED something other than data or ack at DevMac\n";
				delete frame;
				delete msg;
			}
		}
		else if(!isForUs(frame)) //when previous node receives the implicit ACK data packet or a node receives a data packet that is not intended for it
		{
			if (frameType == ST_DATA) {

				Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame *> (frame); //create a frame

				//new packets
				if((!strcmp(address.str().c_str() ,"0A-AA-00-00-00-01")))
				{
					currentpacketSequenceNumber=dframe->getSequenceNumber(); //increase the sequence number, since the first one is 0,
					EV <<"The current sequence(in-coming packet) number is :"<<currentpacketSequenceNumber<<": The previous packet is :" <<previouspacketSequenceNumber <<endl;
				}
				else if((!strcmp(address.str().c_str() ,"0A-AA-00-00-00-02")))
				{
					currentpacketSequenceNumber_2=dframe->getSequenceNumber(); //increase the sequence number, since the first one is 0,
					EV <<"The current sequence(in-coming packet) number is :"<<currentpacketSequenceNumber_2<<": The previous packet is :" <<previouspacketSequenceNumber_2 <<endl;
				}
				else if(!strcmp(address.str().c_str() ,"0A-AA-00-00-00-03"))
				{
					currentpacketSequenceNumber_3=dframe->getSequenceNumber(); //increase the sequence number, since the first one is 0,
					EV <<"The current sequence(in-coming packet) number is :"<<currentpacketSequenceNumber_3<<": The previous packet is :" <<previouspacketSequenceNumber_3 <<endl;
				}
				else if(!strcmp(address.str().c_str() ,"0A-AA-00-00-00-04"))
				{
					currentpacketSequenceNumber_4=dframe->getSequenceNumber(); //increase the sequence number, since the first one is 0,
					EV <<"The current sequence(in-coming packet) number is :"<<currentpacketSequenceNumber_4<<": The previous packet is :" <<previouspacketSequenceNumber_4 <<endl;
				}
				else if((!strcmp(address.str().c_str() ,"0A-AA-00-00-00-05")))
				{
					currentpacketSequenceNumber_5=dframe->getSequenceNumber(); //increase the sequence number, since the first one is 0,
					EV <<"The current sequence(in-coming packet) number is :"<<currentpacketSequenceNumber_5<<": The previous packet is :" <<previouspacketSequenceNumber_5 <<endl;
				}
				else if((!strcmp(address.str().c_str() ,"0A-AA-00-00-00-06")))
				{
					currentpacketSequenceNumber_6=dframe->getSequenceNumber(); //increase the sequence number, since the first one is 0,
					EV <<"The current sequence(in-coming packet) number is :"<<currentpacketSequenceNumber_6<<": The previous packet is :" <<previouspacketSequenceNumber_6 <<endl;
				}
				if( (currentpacketSequenceNumber > previouspacketSequenceNumber) && (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-01")))
				{
					Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame *> (frame); //create a frame
					if(!transmissionQueue.empty()){ EV << transmissionQueue.front()->getSequenceNumber() <<" is at the front of the queue!" <<endl; }
					ev << "RECEIVED IMPLICIT ACK \n"; // trigger another transmission after some time? macState = DevMac::WAITMAC
					ev << "RESET THE BACKOFF TIMER"<<endl;
					if( !transmissionQueue.empty() && (currentpacketSequenceNumber == transmissionQueue.front()->getSequenceNumber())) {
						ev << "POP DATA : " <<currentpacketSequenceNumber << " FROM THE BUFFER !"<< endl;
						transmissionQueue.pop_front(); //pop the front of the transmission que if there is something to pop

						cancelEvent(ackTimeout);
						min_back_off = 0.2;
						previouspacketSequenceNumber = currentpacketSequenceNumber;

						if((strcmp(address.str().c_str(),"0A-AA-00-00-00-01")))
						{
							if(!transmissionQueue.empty())
							{
								sendDataFrame(transmissionQueue.front(), true);
								cancelEvent(ackTimeout);
								macState = DevMac::WAITACK;
								scheduleAt( simTime()+ min_back_off, ackTimeout);
							}
							// 3/12
						}
						else
						{
							cancelEvent(delay_timer_30);
							scheduleAt(simTime()+ protocol_delay , delay_timer_30);		// Here the delay to be added in the protocol
						}



						//if(!transmissionQueue.empty())sendDataFrame(transmissionQueue.front(), true); // 3/12
						//previouspacketSequenceNumber = currentpacketSequenceNumber; //record
						//macState = DevMac::IDLE; //set the state to idle
						//min_back_off = 0.2;
						//cancelEvent(ackTimeout);
						//scheduleAt(simTime() + min_back_off, ackTimeout);//3/12
					}
				}

//////////////////////////////////////////////
				if( (currentpacketSequenceNumber > previouspacketSequenceNumber) && (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-02")))
								{
									Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame *> (frame); //create a frame
									if(!transmissionQueue.empty()){ EV << transmissionQueue.front()->getSequenceNumber() <<" is at the front of the queue!" <<endl; }
									ev << "RECEIVED IMPLICIT ACK \n"; // trigger another transmission after some time? macState = DevMac::WAITMAC
									ev << "RESET THE BACKOFF TIMER"<<endl;
									if( !transmissionQueue.empty() && (currentpacketSequenceNumber_2 == transmissionQueue.front()->getSequenceNumber())) {
										ev << "POP DATA : " <<currentpacketSequenceNumber << " FROM THE BUFFER !"<< endl;
										transmissionQueue.pop_front(); //pop the front of the transmission que if there is something to pop

										cancelEvent(ackTimeout);
										min_back_off = 0.2;
										previouspacketSequenceNumber_2 = currentpacketSequenceNumber_2;

										if((strcmp(address.str().c_str(),"0A-AA-00-00-00-02")))
										{
											if(!transmissionQueue.empty())
											{
												sendDataFrame(transmissionQueue.front(), true);
												cancelEvent(ackTimeout);
												macState = DevMac::WAITACK;
												scheduleAt( simTime()+ min_back_off, ackTimeout);
											}
											// 3/12
										}
										else
										{
											//cancelEvent(delay_timer_30);
											//scheduleAt(simTime()+ 0.04 , delay_timer_30);		// Here the delay to be added in the protocol
										}



										//if(!transmissionQueue.empty())sendDataFrame(transmissionQueue.front(), true); // 3/12
										//previouspacketSequenceNumber = currentpacketSequenceNumber; //record
										//macState = DevMac::IDLE; //set the state to idle
										//min_back_off = 0.2;
										//cancelEvent(ackTimeout);
										//scheduleAt(simTime() + min_back_off, ackTimeout);//3/12
									}
								}
//////////////////////////////////////////////
				else
				{
					ev << "Packet is old or not for us, drop it\n";

				}
				if( (currentpacketSequenceNumber_2 > previouspacketSequenceNumber_2) && (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-02"))  )
				{

					Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame *> (frame); //create a frame
					if(!transmissionQueue.empty()){ EV << transmissionQueue.front()->getSequenceNumber() <<" is at the front of the queue!" <<endl; }
					ev << "RECEIVED IMPLICIT ACK \n"; // trigger another transmission after some time? macState = DevMac::WAITMAC
					ev << "RESET THE BACKOFF TIMER"<<endl;
					if( !transmissionQueue.empty() && (currentpacketSequenceNumber_2 == transmissionQueue.front()->getSequenceNumber()))
					{
						ev << "R-P: " << currentpacketSequenceNumber_2 << " Q-P:" << transmissionQueue.front()->getSequenceNumber() <<endl;
						ev << "POP DATA : " <<currentpacketSequenceNumber_2 << " FROM THE BUFFER !"<< endl;
						transmissionQueue.pop_front(); //pop the front of the transmission que if there is something to pop
						if(!transmissionQueue.empty())sendDataFrame(transmissionQueue.front(), false); // 3/12
						previouspacketSequenceNumber_2 = currentpacketSequenceNumber_2; //record
						//macState = DevMac::IDLE; //set the state to idle
						min_back_off = 0.2;
						cancelEvent(ackTimeout);
						scheduleAt(simTime() + min_back_off, ackTimeout);//3/12
					}
				}
				else
				{
					ev << "Packet is old or not for us, drop it\n";
				}
				if( (currentpacketSequenceNumber_3 > previouspacketSequenceNumber_3) && (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-03"))  ){

					Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame *> (frame); //create a frame
					if(!transmissionQueue.empty()){ EV << transmissionQueue.front()->getSequenceNumber() <<" is at the front of the queue!" <<endl; }
					ev << "RECEIVED IMPLICIT ACK \n"; // trigger another transmission after some time? macState = DevMac::WAITMAC
					ev << "RESET THE BACKOFF TIMER"<<endl;
					if( !transmissionQueue.empty() && (currentpacketSequenceNumber_3 == transmissionQueue.front()->getSequenceNumber())) {
						ev << "R-P: " << currentpacketSequenceNumber_3 << " Q-P:" << transmissionQueue.front()->getSequenceNumber() <<endl;
						ev << "POP DATA : " <<currentpacketSequenceNumber_3 << " FROM THE BUFFER !"<< endl;
						transmissionQueue.pop_front(); //pop the front of the transmission que if there is something to pop
						if(!transmissionQueue.empty())sendDataFrame(transmissionQueue.front(), false); // 3/12
						previouspacketSequenceNumber_3 = currentpacketSequenceNumber_3; //record
						//macState = DevMac::IDLE; //set the state to idle
						min_back_off = 0.2;
						cancelEvent(ackTimeout);
						scheduleAt(simTime() + min_back_off, ackTimeout);//3/12
					}
				}
				else
				{
					ev << "Packet is old or not for us, drop it\n";

				}
				if( (currentpacketSequenceNumber_4 > previouspacketSequenceNumber_4) && (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-04"))  ){

					Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame *> (frame); //create a frame
					if(!transmissionQueue.empty()){ EV << transmissionQueue.front()->getSequenceNumber() <<" is at the front of the queue!" <<endl; }
					ev << "RECEIVED IMPLICIT ACK \n"; // trigger another transmission after some time? macState = DevMac::WAITMAC
					ev << "RESET THE BACKOFF TIMER"<<endl;
					if( !transmissionQueue.empty() && (currentpacketSequenceNumber_4 == transmissionQueue.front()->getSequenceNumber())) {
						ev << "R-P: " << currentpacketSequenceNumber_4 << " Q-P:" << transmissionQueue.front()->getSequenceNumber() <<endl;
						ev << "POP DATA : " <<currentpacketSequenceNumber_4 << " FROM THE BUFFER !"<< endl;
						transmissionQueue.pop_front(); //pop the front of the transmission que if there is something to pop
						if(!transmissionQueue.empty())sendDataFrame(transmissionQueue.front(), false); // 3/12
						previouspacketSequenceNumber_4 = currentpacketSequenceNumber_4; //record
						min_back_off = 0.2;
						cancelEvent(ackTimeout);
						scheduleAt(simTime() + min_back_off, ackTimeout);//3/12
					}
				}
				else
				{
					ev << "Packet is old or not for us, drop it\n";

				}
				if( (currentpacketSequenceNumber_5 > previouspacketSequenceNumber_5) && (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-05"))  ){

					Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame *> (frame); //create a frame
					if(!transmissionQueue.empty()){ EV << transmissionQueue.front()->getSequenceNumber() <<" is at the front of the queue!" <<endl; }
					ev << "RECEIVED IMPLICIT ACK \n"; // trigger another transmission after some time? macState = DevMac::WAITMAC
					ev << "RESET THE BACKOFF TIMER"<<endl;
					if( !transmissionQueue.empty() && (currentpacketSequenceNumber_5 == transmissionQueue.front()->getSequenceNumber())) {
						ev << "R-P: " << currentpacketSequenceNumber_5 << " Q-P:" << transmissionQueue.front()->getSequenceNumber() <<endl;
						ev << "POP DATA : " <<currentpacketSequenceNumber_5 << " FROM THE BUFFER !"<< endl;
						transmissionQueue.pop_front(); //pop the front of the transmission que if there is something to pop
						if(!transmissionQueue.empty())sendDataFrame(transmissionQueue.front(), false); // 3/12
						previouspacketSequenceNumber_5 = currentpacketSequenceNumber_5; //record
						//macState = DevMac::IDLE; //set the state to idle
						min_back_off = 0.2;
						cancelEvent(ackTimeout);
						scheduleAt(simTime() + min_back_off, ackTimeout);//3/12
					}
				}
				else
				{
					ev << "Packet is old or not for us, drop it\n";
				}
				if( (currentpacketSequenceNumber_6 > previouspacketSequenceNumber_6) && (!strcmp(address.str().c_str() ,"0A-AA-00-00-00-06"))  ){
					Ieee80211DataFrame *dframe = dynamic_cast<Ieee80211DataFrame *> (frame); //create a frame
					if(!transmissionQueue.empty()){ EV << transmissionQueue.front()->getSequenceNumber() <<" is at the front of the queue!" <<endl; }
					ev << "RECEIVED IMPLICIT ACK \n"; // trigger another transmission after some time? macState = DevMac::WAITMAC
					ev << "RESET THE BACKOFF TIMER"<<endl;
					if( !transmissionQueue.empty() && (currentpacketSequenceNumber_6 == transmissionQueue.front()->getSequenceNumber())) {
						ev << "R-P: " << currentpacketSequenceNumber_6 << " Q-P:" << transmissionQueue.front()->getSequenceNumber() <<endl;
						ev << "POP DATA : " <<currentpacketSequenceNumber_6 << " FROM THE BUFFER !"<< endl;
						transmissionQueue.pop_front(); //pop the front of the transmission que if there is something to pop
						if(!transmissionQueue.empty())sendDataFrame(transmissionQueue.front(), false); // 3/12
						previouspacketSequenceNumber_6 = currentpacketSequenceNumber_6; //record
						//macState = DevMac::IDLE; //set the state to idle
						min_back_off = 0.2;
						cancelEvent(ackTimeout);
						scheduleAt(simTime() + min_back_off, ackTimeout);//3/12
					}
				}
				else
				{
					ev << "Packet is old or not for us, drop it\n";
				}
				delete frame;
			}
			else {
				if (frameType == ST_ACK ) { //check is the data received is an ack packet, the second node cares about acks,
					ev << "Packet not for us, drop it\n";
					delete frame;
				}
				else{ // all other nodes will drop, really this isn't used at all !!! ignore, whateverz
					ev << "Packet not for us, drop it\n";
					delete frame;
				}
			}
		}
	}

}

Ieee80211DataOrMgmtFrame *DevMac::buildDataFrame(Ieee80211DataOrMgmtFrame *frameToSend)
{
    Ieee80211DataOrMgmtFrame *frame = (Ieee80211DataOrMgmtFrame *)frameToSend->dup();

    return frame;
}
Ieee80211ACKFrame *DevMac::buildACKFrame(Ieee80211DataOrMgmtFrame *frameToACK)
{
    Ieee80211ACKFrame *frame = new Ieee80211ACKFrame("wlan-ack");
    ev << "BUILDING ACK frame, setting receiver address to " <<frameToACK->getTransmitterAddress() << endl;
    frame->setSequenceNumber(frameToACK->getSequenceNumber());
    frame->setReceiverAddress(frameToACK->getTransmitterAddress());
    ev << "The sequence number of the ACK packet inside the buildACKFrame(.) is :" << frame->getSequenceNumber()<<endl;
    return frame;
}

Ieee80211DataOrMgmtFrame *DevMac::buildBroadcastFrame(Ieee80211DataOrMgmtFrame *frameToSend)
{
    Ieee80211DataOrMgmtFrame *frame = (Ieee80211DataOrMgmtFrame *)frameToSend->dup();
    frame->setDuration(0);
    return frame;
}
void DevMac::sendACKFrame(Ieee80211DataOrMgmtFrame *frameToACK)
{
	//basicBitrate=12000; //change this
	basicBitrate=20000; //change this
    EV << "sending ACK frame\n";
    float result = 0;
    //box_muller(0.0001,0, result);



    box_muller(0.00045+0.00014+.00016+0.00375,0.00843, result);
    if(result< 0) result = -1 * result ;

    if(radioState == RadioState::TRANSMIT)
    {
        ev<<"TRANSMITTING ALREADY!"<<endl;
    }
    else if(radioState == RadioState::IDLE){
    	ev<<"IDLE ALREADY!"<<endl;
    }
    else if(radioState == RadioState::RECV){
        ev<<"RECV ALREADY!"<<endl;
    }

    sendDownDelayed(setBasicBitrate(buildACKFrame(frameToACK)),result); // add delay for ACK
}
void DevMac::sendDataFrame(Ieee80211DataOrMgmtFrame *frameToSend, bool delay)
{
	basicBitrate=3*60000; //change this
	float  result = 0;
	//box_muller(0.0015,0, result);
	box_muller(0.00258+0.00053+0.00034+.00326,0.00594, result);
	if(result< 0) result = -1 * result ;
    if(radioState == RadioState::TRANSMIT){
    	ev<<"TRANSMITTING ALREADY!"<<endl;
    }
    else if(radioState == RadioState::IDLE){
    	ev<<"IDLE ALREADY!"<<endl;
    }
    else if(radioState == RadioState::RECV){
    	ev<<"RECV ALREADY!"<<endl;
    }
	if(delay == true){ //first node only
	    EV << "sending Data frame\n";
	    ev << "Random delay chosen is :" << result<<endl;
	    sendDownDelayed(setBasicBitrate(buildDataFrame(frameToSend)), result);
	}
	else{
		EV << "sending Data frame\n";
		sendDownDelayed(setBasicBitrate(buildDataFrame(frameToSend)), result); // add hardware delay here
	}
}
void DevMac::sendBroadcastFrame(Ieee80211DataOrMgmtFrame *frameToSend)
{
    EV << "sending Broadcast frame\n";
    sendDown(buildBroadcastFrame(frameToSend));
}

Ieee80211Frame *DevMac::setBasicBitrate(Ieee80211Frame *frame)
{
    ASSERT(frame->getControlInfo()==NULL);
    PhyControlInfo *ctrl = new PhyControlInfo();
    ctrl->setBitrate(basicBitrate);
    frame->setControlInfo(ctrl);
    return frame;
}

/****************************************************************
 * Helper functions.
 */
void DevMac::finishCurrentTransmission()
{
    popTransmissionQueue();
    resetStateVariables();
}

void DevMac::giveUpCurrentTransmission()
{
    popTransmissionQueue();
    resetStateVariables();
    numGivenUp++;
}

Ieee80211DataOrMgmtFrame *DevMac::getCurrentTransmission()
{
    return (Ieee80211DataOrMgmtFrame *)transmissionQueue.front();
}

void DevMac::sendDownPendingRadioConfigMsg()
{
    if (pendingRadioConfigMsg != NULL)
    {
        sendDown(pendingRadioConfigMsg);
        pendingRadioConfigMsg = NULL;
    }
}

void DevMac::setMode(Mode mode)
{
    if (mode == PCF)
        error("PCF mode not yet supported");
    false;

    this->mode = mode;
}

void DevMac::resetStateVariables()
{
    backoffPeriod = 0;
    retryCounter = 0;

    if (!transmissionQueue.empty()) {
        backoff = true;
        getCurrentTransmission()->setRetry(false);
    }
    else {
        backoff = false;
    }
}

bool DevMac::isMediumStateChange(cMessage *msg)
{
    return msg == mediumStateChange || (msg == endReserve && radioState == RadioState::IDLE);
}

bool DevMac::isMediumFree()
{
    return radioState == RadioState::IDLE; /*&& !endReserve->isScheduled()*/;
}

bool DevMac::isBroadcast(Ieee80211Frame *frame)
{
    return frame && frame->getReceiverAddress().isBroadcast();
}

bool DevMac::isForUs(Ieee80211Frame *frame)
{
    return frame && frame->getReceiverAddress() == address;
}

bool DevMac::isDataOrMgmtFrame(Ieee80211Frame *frame)
{
    return dynamic_cast<Ieee80211DataOrMgmtFrame*>(frame);
}

void DevMac::popTransmissionQueue()
{
    EV << "dropping frame from transmission queue\n";
    Ieee80211Frame *temp = transmissionQueue.front();
    transmissionQueue.pop_front();

    delete temp;

    if (queueModule)
    {
        // tell queue module that we've become idle
        EV << "requesting another frame from queue module\n";
        queueModule->requestPacket();
    }

}

double DevMac::computeFrameDuration(Ieee80211Frame *msg)
{
    return computeFrameDuration(msg->getBitLength(), bitrate);
}

double DevMac::computeFrameDuration(int bits, double bitrate)
{
    return bits / bitrate + PHY_HEADER_LENGTH / BITRATE_HEADER;
}

void DevMac::logState()
{
    EV  << "state information: mode = " << modeName(mode) << ", state = " << fsm.getStateName();

}

const char *DevMac::modeName(int mode)
{
#define CASE(x) case x: s=#x; break
    const char *s = "???";
    switch (mode)
    {
        CASE(DCF);
        CASE(PCF);
    }
    return s;
#undef CASE
}
void DevMac::receiveChangeNotification(int category, const cPolymorphic *details)
{
    Enter_Method_Silent();
    printNotificationBanner(category, details);

    if (category == NF_RADIOSTATE_CHANGED)
    {
        RadioState::State newRadioState = check_and_cast<RadioState *>(details)->getState();

        // FIXME: double recording, because there's no sample hold in the gui
        radioStateVector.record(radioState);
        radioStateVector.record(newRadioState);

        radioState = newRadioState;
    }
}
void DevMac::box_muller(float m, float s, float & result)	/* normal random variate generator */
{				        /* mean m, standard deviation s */
	float x1, x2, w, y1;

	static float y2;
	static int use_last = 0;

	if (use_last)		        /* use value from previous call */
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do {
			x1 = 2.0 * ranf() - 1.0;
			x2 = 2.0 * ranf() - 1.0;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}

	result = m + y1 * s ;
	EV << "m + y1 * s " << result  <<endl;
}
void DevMac::finish()
{
	long throughput = 0;

	ev << "I am Node :" << address.str().c_str() << endl;
	ev << "--------------------------------------------------------" << endl;
	ev << "\t" << getFullPath() << endl;
	ev << "--------------------------------------------------------" << endl;
	if(!strcmp(address.str().c_str(), "0A-AA-00-00-00-01")) ev << "numRetryNode0" << ": "<< numRetryNode0<< endl;
	else if(!strcmp(address.str().c_str(), "0A-AA-00-00-00-02"))ev << "numRetryNode1" << ": " << numRetryNode1 <<endl;
	else if(!strcmp(address.str().c_str(), "0A-AA-00-00-00-03")) ev << "numRetryNode2" << ": " <<numRetryNode2<<endl;
	else if(!strcmp(address.str().c_str(), "0A-AA-00-00-00-04")) ev << "numRetryNode3" << ": " <<numRetryNode3 <<endl;
	else if(!strcmp(address.str().c_str(), "0A-AA-00-00-00-05")) ev << "numRetryNode4" << ": " <<numRetryNode4 <<endl;
	ev <<"--------------------------------------------------------" << endl;
}

