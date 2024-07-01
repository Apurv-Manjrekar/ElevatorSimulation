// Testing elevators

#include <vector>
#include <iostream>
#include "ECElevatorSim.h"

using namespace std;

// Test utility
template<class T>
void ASSERT_EQ(T x, T y)
{
  if( x == y )
  {
    cout << "Test passed: equal: " << x << "  " << y << endl;
  }
  else
  {
    cout << "Test FAILED: equal: " << x << "  " << y << endl;
  }
}

static void RunTest(int numFloors, int timeSim, vector<ECElevatorSimRequest> &listRequests, vector<int> &listArriveTime )
{
    // simulate
    ECElevatorSim sim(numFloors, listRequests );
    sim.Simulate(timeSim);

    // status of requests
    for(unsigned int i=0; i<listRequests.size(); ++i)
    {
        //cout << "Request " << i << ": ";
        int tmDone = listRequests[i].GetArriveTime();
        ASSERT_EQ(tmDone, listArriveTime[i] );
#if 0
        if( tmDone >= 0 )
        {
            cout << "arrived at time " << tmDone;
        }
        else
        {
            cout << " hasn't arrived yet";
        }
        cout << endl;
#endif
    }
}

// a simple test: a single passenger going from floor 3 to 1
// this passenger arrived time 7: 
// (i) elevator gets to floor 3 at time 4 (received request from this passenger at time 2);
// (ii) wait for time 1 to let the passenger in the elevator (now time 5)
// (iii) move to floor 1 from 3, which takes time 6 and 7; so arrive at floor 1 (destination) at time 7
static void Test0()
{
    cout << "\n****** TEST 0\n";
    // test setup
    const int NUM_FLOORS = 7;
    const int timeSim = 10;
    ECElevatorSimRequest r1(2, 3, 1);
    vector<ECElevatorSimRequest> listRequests;
    listRequests.push_back(r1);
    vector<int> listArriveTime;
    listArriveTime.push_back(7);

    // request for elevator at time 2, arrive at time 7
    RunTest(NUM_FLOORS, timeSim, listRequests, listArriveTime);
}

// Two passengers: 
// Passenger 1: arrived time 7: 
// (i) elevator gets to floor 3 at time 4 (received request from this passenger at time 2);
// (ii) wait for time 1 to let the passenger in the elevator (now time 5)
// (iii) move to floor 3 from 5, which takes time 6 and 7; so arrive at floor 5 (destination) at time 7
// Passenger 2: 
// (i) elevator arrives at floor 6 at time 9 (note: unloading at time 8 at floor 5)
// (ii) move downwards starting at time 10; take 6 time units to get to floor 1 (arrived at floor 1 at time 15)
static void Test1()
{
    cout << "\n****** TEST 1\n";
    // test setup
    const int NUM_FLOORS = 7;
    const int timeSim = 20;
    ECElevatorSimRequest r1(2, 3, 5), r2(2, 6, 1);
    vector<ECElevatorSimRequest> listRequests;
    listRequests.push_back(r1);
    listRequests.push_back(r2);
    vector<int> listArriveTime;
    listArriveTime.push_back(7);
    listArriveTime.push_back(15);

    // simulate
    RunTest(NUM_FLOORS, timeSim, listRequests, listArriveTime);
}

// Two passengers: 
// (i) elevator gets to floor 4 at time 5 to pick up passenger 1 (received request from this passenger at time 2);
// (ii) wait for time 1 to let passenger 1 in the elevator (now time 6)
// (iii) move to floor 4 from 5 (arrived at time 7), so to pick up passenger 2; 
// (iv) at time 8 moving downwards from floor 5
// (v) elevator arrives at floor 2 at time 11 (so to unload passenger 2; i.e., passenger 2 arrives at time 11)
// (vi) move downwards at time 12 and arrive at floor 1 at time 13; unload passenger 1 (i.e. passenger 1 arrives at time 13)
static void Test2()
{
    cout << "\n****** TEST 2\n";
    // test setup
    const int NUM_FLOORS = 7;
    const int timeSim = 20;
    ECElevatorSimRequest r1(2, 4, 1), r2(3, 5, 2);
    vector<ECElevatorSimRequest> listRequests;
    listRequests.push_back(r1);
    listRequests.push_back(r2);
    vector<int> listArriveTime;
    listArriveTime.push_back(13);
    listArriveTime.push_back(11);

    // simulate
    RunTest(NUM_FLOORS, timeSim, listRequests, listArriveTime);
}

// Three passengers: 
// (i) elevator starts to move upwards from floor 1 at time 2 
// (ii) pick up passenger 2 at time 3 at floor 2 (still going up)
// (iii) arrive at floor 4 at time 6, so to pick up passenger 1; 
// (iv) at time 8 arrives at floor 5 to unload passenger 2 (passenger 2 arrives at time 8) and moving downwards at time 9 (note: passenger 3 is not ready to go yet)
// (v) arrives at floor 1 at time 13 (so to unload passenger 1; i.e., passenger 1 arrives at time 13)
// (vi) move upwards starting at time 14 and arrive at floor 5 at time 18 and pick up passenger 3 
// (vii) move downwards starting at time 19 and arrives at floor 1 at time 23 (passenger 3 arrives at time 23)
static void Test3()
{
    cout << "\n****** TEST 3\n";
    // test setup
    const int NUM_FLOORS = 8;
    const int timeSim = 25;
    ECElevatorSimRequest r1(2, 4, 1), r2(3, 2, 5), r3(12, 5, 1);
    vector<ECElevatorSimRequest> listRequests;
    listRequests.push_back(r1);
    listRequests.push_back(r2);
    listRequests.push_back(r3);
    vector<int> listArriveTime;
    listArriveTime.push_back(13);
    listArriveTime.push_back(8);
    listArriveTime.push_back(23);

    // simulate
    RunTest(NUM_FLOORS, timeSim, listRequests, listArriveTime);
}

// Four passengers: 
// passengers 1 and 2 both arrives at time 13
// Floor 3 to 1 at time 2, floor 5 to 1 at time 3
// passenger 3 arrives at time 16
// floor 2 to 3 at tiem 8
// passenger 4 arrives at time 26
// floor 6 to 1 at time 10
static void Test4()
{
    cout << "\n****** TEST 4\n";
    // test setup
    const int NUM_FLOORS = 8;
    const int timeSim = 35;
    ECElevatorSimRequest r1(2, 3, 1), r2(3, 5, 1), r3(8, 2, 3), r4(10, 6, 1);
    vector<ECElevatorSimRequest> listRequests;
    listRequests.push_back(r1);
    listRequests.push_back(r2);
    listRequests.push_back(r3);
    listRequests.push_back(r4);
    vector<int> listArriveTime;
    listArriveTime.push_back(13);
    listArriveTime.push_back(13);
    listArriveTime.push_back(16);
    listArriveTime.push_back(26);

    // simulate
    RunTest(NUM_FLOORS, timeSim, listRequests, listArriveTime);
}

static void Test5()
{
    cout << "\n****** TEST 5\n";
    // test setup
    const int NUM_FLOORS = 10;
    const int timeSim = 25;
    ECElevatorSimRequest r1(1, 3, 1), r2(12, 5, 4);
    vector<ECElevatorSimRequest> listRequests;
    listRequests.push_back(r1);
    listRequests.push_back(r2);
    vector<int> listArriveTime;
    listArriveTime.push_back(6);
    listArriveTime.push_back(18);

    // simulate
    RunTest(NUM_FLOORS, timeSim, listRequests, listArriveTime);
}

static void Test6()
{
    cout << "\n****** TEST 6\n";
    // test setup
    const int NUM_FLOORS = 8;
    const int timeSim = 25;
    ECElevatorSimRequest r1(1, 2, 3), r2(3, 2, 3);
    vector<ECElevatorSimRequest> listRequests;
    listRequests.push_back(r1);
    listRequests.push_back(r2);
    vector<int> listArriveTime;
    listArriveTime.push_back(4);
    listArriveTime.push_back(8);

    // simulate
    RunTest(NUM_FLOORS, timeSim, listRequests, listArriveTime);
}

static void Test10()
{
    const int NUM_FLOORS = 3;

    const int timeSim = 50;

    ECElevatorSimRequest r1(1, 3, 1), r2(2, 3, 2), r3(10, 2, 3), r4(14, 2, 1), r5(20, 3, 2), r6(30, 3, 1), r7(34, 3, 2), r8(26, 2, 3), r9(28, 2, 1), r10(16, 3, 2);

    vector<ECElevatorSimRequest> listRequests;

    listRequests.push_back(r1);

    listRequests.push_back(r2);

    listRequests.push_back(r3);

    listRequests.push_back(r4);

    listRequests.push_back(r5);

    listRequests.push_back(r6);

    listRequests.push_back(r7);

    listRequests.push_back(r8);

    listRequests.push_back(r9);

    listRequests.push_back(r10);

    vector<int> listArriveTime;

    listArriveTime.push_back(7);

    listArriveTime.push_back(5);

    listArriveTime.push_back(13);

    listArriveTime.push_back(17);

    listArriveTime.push_back(22);

    listArriveTime.push_back(38);

    listArriveTime.push_back(36);

    listArriveTime.push_back(27);

    listArriveTime.push_back(31);

    listArriveTime.push_back(22);



    // simulate

    RunTest(NUM_FLOORS, timeSim, listRequests, listArriveTime);
}


int main()
{
    Test0();
    Test1();
    Test2();
    Test3();
    Test4();
    Test5();
    Test10();
    Test6();
}
