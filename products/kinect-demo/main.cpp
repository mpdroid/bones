/*
* Demonstrates Azure Kinect DK features.
* Acknowledgements to following repos:
*/

using namespace std;
#include "common.h"
#include "controller.h"
using namespace controller;

int preflightChecks() {
    char *visionKey = getenv("AZURE_VISION_KEY"); 
    if (visionKey == NULL) {
        ERROR("Obtain Azure Vision subscription key and set AZURE_VISION_KEY in OS environment.");
        return 1;
    }
    char *visionEndpoint = getenv("AZURE_VISION_ENDPOINT"); 
    if (visionEndpoint == NULL) {
        ERROR("Obtain Azure Vision endpoint and set AZURE_VISION_ENDPOINT in OS environment.");
        return 1;
    }
    return 0;
}

void printUsage() {
    cout << "Press 'J' to display joint information..." << endl;
    cout << "Press 'L' to switch to Light Sabers..." << endl;
    cout << "Press 'O' to switch to Object detection; Point with right hand to trigger detection..." << endl;
    cout << "Press 'W' to writing mode; raise left hand above your head and start writing with your right..." << endl;
}

// TODO
// handle out of window conditions
// improve accuracy at close range
int main()
{
    if (preflightChecks() != 0)
        return 1;
    printUsage();
    Controller *controller = Controller::getInstance();
    int retval = controller->runLoop();
    cout << "so long..." << endl;
    return retval;
}
