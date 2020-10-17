# kinect-dk-demo
Demonstrates Kinect DK camera body tracking features.  Watch this youtube video to see the features in action.


## Usage
Note: This project will developed on Ubuntu 20.04. Do not have the infrastructue to verify it works on other Linux versions and Windows.

### Step 1: [Buy an Azure Kinect DK camera](https://www.microsoft.com/en-us/p/azure-kinect-dk/8pp5vxmd9nhq?rtc=1&activetab=pivot:overviewtab)

### Step 2: Install the libraries
The general steps are as outlined in [Microsoft documentation](https://docs.microsoft.com/en-us/azure/Kinect-dk/sensor-sdk-download); but with a couple of hacks to make things work on Ubuntu 20.04.
- use of 18.04 repo, even though OS is 20.04
- installed lower versions of tools and libraries (as latest versions of sensor and body tracker don't seem to be compatible on 20.04) 
```
$ curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
$ sudo apt-add-repository https://packages.microsoft.com/ubuntu/18.04/prod
$ curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | sudo tee /etc/apt/sources.list.d/microsoft-prod.list
$ curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
$ sudo apt-get update
$ sudo apt install libk4a1.3-dev
$ sudo apt install libk4abt1.0-dev
$ sudo apt install k4a-tools=1.3.0

````
- Verify sensor library by launching camera viewer
````
$ k4aviewer
````

### Step 3: Other pre-requisites
1. Gnu C Compiler(gcc 9.3.0+)

2. cmake
````sudo apt-get install cmake````

2. ninja-build
````sudo apt-get install ninja-build````

3. Eigen3
````sudo apt-get install libeigen3-dev````

4. Obtain a Azure Vision subscription key and store it the environment variable `AZURE_VISION_KEY`

### Step 4: Clone this project

````
$ git clone --recursive https://github.com/mpdroid/kinect-dk-demo
````
- cilantro will also be cloned as a submodule in products/extern/cilantro

### Step 5: Build and run  
````
$ cd kinect-dk-demo
$ mkdir build
$ cd build
$ cmake .. -GNinja
$ ninja
$ ./bin/kinector
````
If all has gone well, you should see the below instructions flash by followed by the appearance of your own mug in an application window.
- Press 'L' for light sabers...
- Press 'O' for object detection; Point with right hand to trigger detection...
- Press 'W' for air-writing; raise left hand above your head and start writing with your right...
- Press 'J' to display joint information...

## How it works
*Under construction*



## Ackowledgments and References
- [Azure-Kinect-Sensor-SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) - Basics of camera capture and rendering in 2D and 3D
- [Azure-Kinect-Samples](https://github.com/microsoft/Azure-Kinect-Samples) - Advanced examples including body tracking
- [kzampog/cilantro](https://github.com/kzampog/cilantro) - Point Cloud manipulation including clustering
- [ocurnut/imgui](https://github.com/ocornut/imgui) - Rendering depth and camera images with drawing overlays
- [deercoder/cpprestsdk-example](https://github.com/deercoder/cpprestsdk-example) - Using cpprestsdk consume Azure vision services
- [Note on Ray-Plane intersection - by Sam Simons](https://samsymons.com/blog/math-notes-ray-plane-intersection/)
