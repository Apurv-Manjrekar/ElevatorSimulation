//
//  ECElevatorSim.h
//  


#ifndef ECElevatorSim_h
#define ECElevatorSim_h

#include <iostream>
#include <set>
#include <vector>
#include <map>
#include <string>

//*****************************************************************************
// DON'T CHANGE THIS CLASS
// 
// Elevator simulation request: 
// (i) time: when the request is made
// (ii) floorSrc: which floor the user is at at present
// (iii) floorDest floor: where the user wants to go; we assume floorDest != floorSrc
// 
// Note: a request is in three stages:
// (i) floor request: the passenger is waiting at floorSrc; once the elevator arrived 
// at the floor (and in the right direction), move to the next stage
// (ii) inside request: passenger now requests to go to a specific floor once inside the elevator
// (iii) Once the passenger arrives at the floor, this request is considered to be "serviced"
//
// two sspecial requests:
// (a) maintenance start: floorSrc=floorDest=-1; put elevator into maintenance 
// starting at the specified time; elevator starts at the current floor
// (b) maintenance end: floorSrc=floorDest=0; put elevator back to operation (from the current floor)

class ECElevatorSimRequest
{
public:
    ECElevatorSimRequest(int timeIn, int floorSrcIn, int floorDestIn) : time(timeIn), floorSrc(floorSrcIn), floorDest(floorDestIn), fFloorReqDone(false), fServiced(false), timeArrive(-1) {} 
    ECElevatorSimRequest(const ECElevatorSimRequest &rhs) : time(rhs.time), floorSrc(rhs.floorSrc), floorDest(rhs.floorDest), fFloorReqDone(rhs.fFloorReqDone), fServiced(rhs.fServiced), timeArrive(rhs.timeArrive) {}
    int GetTime() const {return time; }
    int GetFloorSrc() const { return floorSrc; }
    int GetFloorDest() const { return floorDest; }
    bool IsGoingUp() const { return floorDest >= floorSrc; }

    // Is this passenger in the elevator or not
    bool IsFloorRequestDone() const { return fFloorReqDone; }
    void SetFloorRequestDone(bool f) { fFloorReqDone = f; }

    // Is this event serviced (i.e., the passenger has arrived at the desstination)?
    bool IsServiced() const { return fServiced; }
    void SetServiced(bool f) { fServiced = f; }

    // Get the floor to service
    // If this is in stage (i): waiting at a floor, return that floor waiting at
    // If this is in stage (ii): inside an elevator, return the floor going to
    // Otherwise, return -1
    int GetRequestedFloor() const {
        if( IsServiced() )  {
            return -1;
        }
        else if( IsFloorRequestDone() )   {
            return GetFloorDest();
        }
        else {
            return GetFloorSrc();
        }
    }

    // Wait time: get/set. Note: you need to maintain the wait time yourself!
    int GetArriveTime() const { return timeArrive; }
    void SetArriveTime(int t) { timeArrive = t; }

    // Check if this is the special maintenance start request
    bool IsMaintenanceStart() const { return floorSrc==-1 && floorDest==-1; }
    bool IsMaintenanceEnd() const { return floorSrc==0 && floorDest==0; }

private:
    int time;           // time of request made
    int floorSrc;       // which floor the request is made
    int floorDest;      // which floor is going
    bool fFloorReqDone;   // is this passenger passing stage one (no longer waiting at the floor) or not
    bool fServiced;     // is this request serviced already?
    int timeArrive;     // when the user gets to the desitnation floor
};

//*****************************************************************************
// Elevator moving direction

typedef enum
{
    EC_ELEVATOR_STOPPED = 0,    // not moving
    EC_ELEVATOR_UP,             // moving up
    EC_ELEVATOR_DOWN            // moving down
} EC_ELEVATOR_DIR;

//*****************************************************************************
// Add your own classes here...
class ECElevatorState;

//*****************************************************************************
// Simulation of elevator

class ECElevatorSim
{
public:
    // numFloors: number of floors serviced (floors numbers from 1 to numFloors)
    ECElevatorSim(int numFloors, std::vector<ECElevatorSimRequest> &listRequests);

    // free buffer
    ~ECElevatorSim();

    // Simulate by going through all requests up to certain period of time (as specified in lenSim)
    // starting from time 0. For example, if lenSim = 10, simulation stops at time 10 (i.e., time 0 to 9)
    // Caution: the list of requests contain all requests made at different time;
    // at a specific time of simulation, some events may be made in the future (which you shouldn't consider these future requests)
    void Simulate(int lenSim);

    // The following methods are about querying/setting states of the elevator
    // which include (i) number of floors of the elevator, 
    // (ii) the current floor: which is the elevator at right now (at the time of this querying). Note: we don't model the tranisent states like when the elevator is between two floors
    // (iii) the direction of the elevator: up/down/not moving

    // Get num of floors
    int GetNumFloors() const;

    // Get current floor
    int GetCurrFloor() const;

    // Set current floor
    void SetCurrFloor(int f);

    // Get current direction
    EC_ELEVATOR_DIR GetCurrDir() const;

    // Set current direction
    void SetCurrDir(EC_ELEVATOR_DIR dir);

    // Get current time
    int GetTime() const;

    // Set current time
    void SetTime(int t);

    // Get the list of requests
    std::vector<ECElevatorSimRequest>& GetListRequests() const;

    // Change state
    void ChangeState(ECElevatorState* newState);
    
    // Get current state
    ECElevatorState* GetCurrState() const;

private:
    int numFloors; // number of floors
    int currFloor; // current floor
    int time; // current time
    EC_ELEVATOR_DIR currDir; // current direction
    std::vector<ECElevatorSimRequest>& listRequests; // list of requests
    ECElevatorState* currState; // current state
};


//*****************************************************************************
// Elevator States

class ECElevatorState
{
public:
    // true virtual functions
    virtual void NextDecision(ECElevatorSim& elevatorSim) = 0;
    virtual EC_ELEVATOR_DIR GetDirection() const = 0;
protected:
    // virtual functions
    virtual std::vector<ECElevatorSimRequest*>& GetActiveRequests(ECElevatorSim& elevatorSim, std::vector<ECElevatorSimRequest*>& activeRequests) const;
    virtual bool DoCurrentFloorRequests(ECElevatorSim& elevatorSim, std::vector<ECElevatorSimRequest*>& requests) const;

};

class ECElevatorStopped : public ECElevatorState
{
    // Get closest requested floor
    int GetClosestRequestFloor(ECElevatorSim& elevatorSim, std::vector<ECElevatorSimRequest*>& activeRequests) const;
    // Do next decision
    virtual void NextDecision(ECElevatorSim& elevatorSim);
    // Get direction
    virtual EC_ELEVATOR_DIR GetDirection() const;
};

class ECElevatorUp : public ECElevatorState
{
    // Check if request on path
    bool RequestOnPath(ECElevatorSim& elevatorSim, std::vector<ECElevatorSimRequest*>& activeRequests) const;
    // Do next decision
    virtual void NextDecision(ECElevatorSim& elevatorSim);
    // Get direction
    virtual EC_ELEVATOR_DIR GetDirection() const;
};

class ECElevatorDown : public ECElevatorState
{
    // Check if request on path
    bool RequestOnPath(ECElevatorSim& elevatorSim, std::vector<ECElevatorSimRequest*>& activeRequests) const;
    // Do next decision
    virtual void NextDecision(ECElevatorSim& elevatorSim);
    // Get direction
    virtual EC_ELEVATOR_DIR GetDirection() const;
};

#endif /* ECElevatorSim_h */
