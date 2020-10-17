/*
* Demonstrates Azure Kinect DK features.
* Acknowledgements to following repos:
*/

using namespace std;
#include "common.h"
#include "controller.h"
using namespace controller;

void printUsage() {
    cout << "Press 'J' to display joint information..." << endl;
    cout << "Press 'L' to switch to Light Sabers..." << endl;
    cout << "Press 'O' to switch to Object detection; Point with right hand to trigger detection..." << endl;
    cout << "Press 'W' to writing mode; raise left hand above your head and start writing with your right..." << endl;
}

// TODO warning for Azure Vision Key
// Draw all joints with connections
// handle out of window conditions
// improve accuracy at close range
int main()
{
    printUsage();
    Controller *controller = Controller::getInstance();
    int retval = controller->runLoop();
    cout << "so long..." << endl;
    return retval;
}
