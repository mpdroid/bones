/*
* Demonstrates Azure Kinect DK features.
* Acknowledgements to following repos:
*/

using namespace std;
#include "common.h"
#include "controller.h"
using namespace controller;

int main()
{
    Controller *controller = Controller::getInstance();
    int retval = controller->runLoop();
    cout << "so long..." << endl;
    return retval;
}
