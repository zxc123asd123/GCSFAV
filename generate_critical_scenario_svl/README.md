# Generate Scenarios For Testing Apollo in SVL Simulator


This project contains the implementation of our generation approach for constructing virtual scenarios to test Apollo in SVL simulator. This approach generates scenarios by extracted influential factors of traffic accidents. 

The generation approach requires the following dependencies to run:


	1. SVL simulator: https://www.svlsimulator.com/
	
	2. Apollo autonomous driving platform: https://github.com/ApolloAuto/apollo


# Prerequisites

* A 8-core processor and 16GB memory minimum
* Ubuntu 18.04 or later
* Python 3.6 or higher
* NVIDIA graphics card: NVIDIA proprietary driver (>=455.32) must be installed
* CUDA upgraded to version 11.1 to support Nvidia Ampere (30x0 series) GPUs
* Docker-CE version 19.03 and above
* NVIDIA Container Toolkit


# SVL - A Python API for SVL Simulator

Documentation is available on: https://www.svlsimulator.com/docs/

# Apollo - A high performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Vehicles

Website of Apollo: https://apollo.auto/

Installation of Apollo5.0: https://www.svlsimulator.com/docs/system-under-test/apollo5-0-instructions/

# Run

The demonstration of our approach to generate distinct types of safety violation scenarios is placed in the "examples of distinct safety violation scenarios" folder.

To run the approach, execute the main() method of generate_test_case.py in generation_scenario directory; to stop the running, please press Ctrl+C until the program exits.
To stop the approach, press and hold control + c until the program exists the loop.


