/*
* Demonstrates kinect features.
* Acknowledgements to following repos:
* ....
* TODO
* - list repos
* - stick to common method and variable naming convention
* Actors
*   - main
*   - kinect_demo_viewer
*   - controller
    - scene
        - saber
        - writair
        - thing-finder
*   - kinector
*   - euclid
*/

using namespace std;

#include "controller.h"
using namespace controller;

int main()
{
    Controller *controller = Controller::getInstance();
    return controller->runLoop();
}
