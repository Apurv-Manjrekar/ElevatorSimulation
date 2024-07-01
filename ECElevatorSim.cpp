//
//  ECElevatorSim.cpp
//  


#include "ECElevatorSim.h"
#include <vector>

using namespace std;

// *************************************************************************************************
// ECElevatorSim

// numFloors: number of floors serviced (floors numbers from 1 to numFloors)
ECElevatorSim::ECElevatorSim(int numFloors, std::vector<ECElevatorSimRequest> &listRequests) : listRequests(listRequests)
{
    this->numFloors = numFloors; // set num floors
    currFloor = 1; // set current floor to 1 (elevator starts at floor 1)
    currDir = EC_ELEVATOR_STOPPED; // set current direction to stopped
    currState = new ECElevatorStopped(); // set current state to stopped
    time = 0; // set time to 0
}

// free buffer
ECElevatorSim::~ECElevatorSim()
{
    delete currState; // free memory for current state
}

// Simulate by going through all requests up to certain period of time (as specified in lenSim)
// starting from time 0. For example, if lenSim = 10, simulation stops at time 10 (i.e., time 0 to 9)
// Caution: the list of requests contain all requests made at different time;
// at a specific time of simulation, some events may be made in the future (which you shouldn't consider these future requests)
void ECElevatorSim::Simulate(int lenSim)
{
    // iterate through time
    for(int time = 0; time < lenSim; time++)
    {
        currState->NextDecision(*this); // do next decision for that time
    }
}

// The following methods are about querying/setting states of the elevator
// which include (i) number of floors of the elevator, 
// (ii) the current floor: which is the elevator at right now (at the time of this querying). Note: we don't model the tranisent states like when the elevator is between two floors
// (iii) the direction of the elevator: up/down/not moving

// Get num of floors
int ECElevatorSim::GetNumFloors() const { return numFloors; }

// Get current floor
int ECElevatorSim::GetCurrFloor() const { return currFloor; }

// Set current floor
void ECElevatorSim::SetCurrFloor(int f)
{
    currFloor = f;
}

// Get current direction
EC_ELEVATOR_DIR ECElevatorSim::GetCurrDir() const { return currDir; }

// Set current direction
void ECElevatorSim::SetCurrDir(EC_ELEVATOR_DIR dir)
{
    currDir = dir;
}

// Get time
int ECElevatorSim::GetTime() const { return time; }

// Set time
void ECElevatorSim::SetTime(int t)
{
    time = t;
}

// Get current state
ECElevatorState* ECElevatorSim::GetCurrState() const { return currState; }

// Get list of requests
vector<ECElevatorSimRequest>& ECElevatorSim::GetListRequests() const { return listRequests; }

// Change state
void ECElevatorSim::ChangeState(ECElevatorState* newState)
{
    delete currState;
    currState = newState;
}

// *************************************************************************************************
// ECElevatorState

// Get active requests (requests that are not done and are made before current time)
vector<ECElevatorSimRequest*>& ECElevatorState::GetActiveRequests(ECElevatorSim& elevatorSim, vector<ECElevatorSimRequest*>& activeRequests) const
{
    activeRequests.clear(); // clear active requests
    // iterate through list of requests
    for(int i = 0; i < elevatorSim.GetListRequests().size(); i++)
    {
        // if request is made before current time and not serviced, add to active requests
        if(elevatorSim.GetListRequests()[i].GetTime() <= elevatorSim.GetTime() && !elevatorSim.GetListRequests()[i].IsServiced())
        {
            activeRequests.push_back(&elevatorSim.GetListRequests()[i]);
        }
    }
    return activeRequests; // return active requests
}

// Do current floor requests (either someone wants to get off or enter on this floor)
bool ECElevatorState::DoCurrentFloorRequests(ECElevatorSim& elevatorSim, vector<ECElevatorSimRequest*>& requests) const
{
    bool requestFound = false; // bool to keep track of any request found
    // iterate through list of requests
    for(int i = 0; i < requests.size(); i++)
    {
        // if request source on this floor and not picked up
        if(requests[i]->GetFloorSrc() == elevatorSim.GetCurrFloor() && !requests[i]->IsFloorRequestDone())
        {
            requestFound = true;
            requests[i]->SetFloorRequestDone(true); // mark request as picked up
        }
        // if request destination on this floor and picked up (on elevator)
        else if(requests[i]->GetFloorDest() == elevatorSim.GetCurrFloor() && requests[i]->IsFloorRequestDone())
        {
            requestFound = true;
            requests[i]->SetArriveTime(elevatorSim.GetTime()); // set arrive time
            requests[i]->SetServiced(true); // mark request as serviced
            requests.erase(requests.begin() + i); // remove request from active requests
            i--; // decrement i (to account for the removed request)
        }
    }
    return requestFound; // return if request completed
}

// *************************************************************************************************
// ECElevatorStopped

// Get closest request floor
int ECElevatorStopped::GetClosestRequestFloor(ECElevatorSim& elevatorSim, vector<ECElevatorSimRequest*>& activeRequests) const
{
    int minDist = elevatorSim.GetCurrFloor() + elevatorSim.GetNumFloors(); // var to keep track of dist to closest floor request
    int closestFloor = 0; // var to keep track of closest floor
    int currFloor = 0; // var to keep track of current floor
    int currDist = 0; // var to keep track of dist to current floor
    // iterate through active requests
    for(auto& request : activeRequests)
    {
        // if already picked up (in elevator), compare dest floor to closest floor
        if(request->IsFloorRequestDone())
        {
            currDist = abs(request->GetFloorDest() - elevatorSim.GetCurrFloor()); // get dist to dest floor
            currFloor = request->GetFloorDest(); // get dest floor
        }
        else
        {
            currDist = abs(request->GetFloorSrc() - elevatorSim.GetCurrFloor()); // get dist to src floor
            currFloor = request->GetFloorSrc(); // get src floor
        }
        // check if current floor closer than closest floor
        if(currDist < minDist)
        {
            minDist = currDist;
            closestFloor = currFloor;
        }
        // check if current floor equal to closest floor (and going up)
        else if(currDist == minDist && currFloor > elevatorSim.GetCurrFloor())
        {
            closestFloor = currFloor;
        }
    }
    return closestFloor; // return closest floor
}

void ECElevatorStopped::NextDecision(ECElevatorSim& elevatorSim)
{
    cout << "Stopped: " << elevatorSim.GetTime() << " " << elevatorSim.GetCurrFloor() << endl;
    vector<ECElevatorSimRequest*> activeRequests; // var to hold active requests
    GetActiveRequests(elevatorSim, activeRequests); // get active requests
    // if there are active requests start moving to them
    if(activeRequests.size() > 0) 
    {
        // do requests on current floor
        DoCurrentFloorRequests(elevatorSim, activeRequests);
        // move to closest request (change state and floor appropriately)
        if(GetClosestRequestFloor(elevatorSim, activeRequests) > elevatorSim.GetCurrFloor())
        {
            elevatorSim.ChangeState(new ECElevatorUp());
            elevatorSim.SetCurrFloor(elevatorSim.GetCurrFloor() + 1);
        }
        else
        {
            elevatorSim.ChangeState(new ECElevatorDown());
            elevatorSim.SetCurrFloor(elevatorSim.GetCurrFloor() - 1);
        }
        elevatorSim.SetCurrDir(elevatorSim.GetCurrState()->GetDirection()); // update curr direction
    }
    elevatorSim.SetTime(elevatorSim.GetTime() + 1); // increment time
}

// Get direction 
EC_ELEVATOR_DIR ECElevatorStopped::GetDirection() const { return EC_ELEVATOR_STOPPED; }

// *************************************************************************************************
// ECElevatorUp

// Check if request on current path/direction
bool ECElevatorUp::RequestOnPath(ECElevatorSim& elevatorSim, vector<ECElevatorSimRequest*>& activeRequests) const
{
    // iterate through active requests
    for(auto& request : activeRequests)
    {
        // if request source floor on path and not picked up, return true
        if((request->GetFloorSrc() > elevatorSim.GetCurrFloor()) && !(request->IsFloorRequestDone()))
        {
            return true;
        }
        // if request destination floor on path and picked up, return true
        else if((request->GetFloorDest() > elevatorSim.GetCurrFloor()) && request->IsFloorRequestDone())
        {
            return true;
        }
    }
    return false; // return false if no requests on path
}

// Do next decision (going up)
void ECElevatorUp::NextDecision(ECElevatorSim& elevatorSim)
{
    cout << "Up: " << elevatorSim.GetTime() << " " << elevatorSim.GetCurrFloor() << endl;
    vector<ECElevatorSimRequest*> activeRequests; // var to hold active requests
    GetActiveRequests(elevatorSim, activeRequests); // get active requests
    // if no requests, change state to stopped
    if(activeRequests.size() == 0)
    {
        elevatorSim.ChangeState(new ECElevatorStopped());
        elevatorSim.SetCurrDir(elevatorSim.GetCurrState()->GetDirection()); // update curr direction
        elevatorSim.SetTime(elevatorSim.GetTime() + 1); // increment time
        return;
    }
    // else if there are active requests
    // do requests on current floor (if not already loaded/unloaded)
    if(elevatorSim.GetCurrDir() == EC_ELEVATOR_STOPPED || !DoCurrentFloorRequests(elevatorSim, activeRequests))
    {
        // if no requests on current floor (or already loaded/unloaded), check if request on path, if yes keep moving on path, else change direction
        if(RequestOnPath(elevatorSim, activeRequests))
        {
            elevatorSim.SetCurrFloor(elevatorSim.GetCurrFloor() + 1);
        }
        else
        {
            elevatorSim.ChangeState(new ECElevatorDown());
            elevatorSim.SetCurrFloor(elevatorSim.GetCurrFloor() - 1);
        }
        elevatorSim.SetCurrDir(elevatorSim.GetCurrState()->GetDirection()); // update curr direction
    }
    // if request on current floor, set direction temporarily to stopped to indicate already loaded/unloaded
    else
    {
        elevatorSim.SetCurrDir(EC_ELEVATOR_STOPPED);
    }
    elevatorSim.SetTime(elevatorSim.GetTime() + 1); // increment time
}

// Get direction
EC_ELEVATOR_DIR ECElevatorUp::GetDirection() const { return EC_ELEVATOR_UP; }

// *************************************************************************************************
// ECElevatorDown

// Check if request on current path
bool ECElevatorDown::RequestOnPath(ECElevatorSim& elevatorSim, vector<ECElevatorSimRequest*>& activeRequests) const
{
    // iterate through active requests
    for(auto& request : activeRequests)
    {
        // if request source floor on path and not picked up, return true
        if((request->GetFloorSrc() < elevatorSim.GetCurrFloor()) && !(request->IsFloorRequestDone()))
        {
            return true;
        }
        // if request destination floor on path and picked up, return true
        else if((request->GetFloorDest() < elevatorSim.GetCurrFloor()) && request->IsFloorRequestDone())
        {
            return true;
        }
    }
    return false; // return false if no requests on path
}

// Do next decision (going down)
void ECElevatorDown::NextDecision(ECElevatorSim& elevatorSim)
{
    cout << "Down: " << elevatorSim.GetTime() << " " << elevatorSim.GetCurrFloor() << endl;
    vector<ECElevatorSimRequest*> activeRequests; // var to hold active requests
    GetActiveRequests(elevatorSim, activeRequests); // get active requests
    // if no requests, change state to stopped
    if(activeRequests.size() == 0)
    {
        elevatorSim.ChangeState(new ECElevatorStopped());
        elevatorSim.SetCurrDir(elevatorSim.GetCurrState()->GetDirection()); // update curr direction
        elevatorSim.SetTime(elevatorSim.GetTime() + 1); // increment time
        return;
    }
    // If there are active requests
    // do requests on current floor (if not already loaded/unloaded)
    if(elevatorSim.GetCurrDir() == EC_ELEVATOR_STOPPED || !DoCurrentFloorRequests(elevatorSim, activeRequests))
    {
        // if no requests on current floor (or already loaded/unloaded), check if request on path, if yes keep moving on path, else change direction
        if(RequestOnPath(elevatorSim, activeRequests))
        {
            elevatorSim.SetCurrFloor(elevatorSim.GetCurrFloor() - 1);
        }
        else
        {
            elevatorSim.ChangeState(new ECElevatorUp());
            elevatorSim.SetCurrFloor(elevatorSim.GetCurrFloor() + 1);
        }
        elevatorSim.SetCurrDir(elevatorSim.GetCurrState()->GetDirection()); // update curr direction
    }
    // if request on current floor, set direction temporarily to stopped to indicate already loaded/unloaded
    else
    {
        elevatorSim.SetCurrDir(EC_ELEVATOR_STOPPED);
    }
    elevatorSim.SetTime(elevatorSim.GetTime() + 1); // increment time
}

// Get direction
EC_ELEVATOR_DIR ECElevatorDown::GetDirection() const { return EC_ELEVATOR_DOWN; }
