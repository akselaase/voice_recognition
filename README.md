# voice_recognition
Node to perform keyword detection on an audio stream, and publish recognised words on a ROS topic.

## Installation
1. Clone this repo to your catkin workspace src folder, and change to the `voice_recognition` directory.
2. Install dependencies. Run `bin/check_deps.sh` to see which packages are required. 
3. Sox can be installed from apt: `sudo apt install sox`
4. The python packages can be installed with pip (which can be installed with `sudo apt install python3-pip`). 
In the steps below, choose either `tensorflow` or `tensorflow-gpu` depending on your device. CUDA and CUDNN must be properly setup for GPU acceleration to work.

	4a) If installing globally, execute `pip3 install --user pyyaml rospkg rospy std_msgs tensorflow matplotlib`.

	4b) If installing locally, follow these steps as you see fit.
	1. Install the virtualenv package:
	`sudo apt install python3-venv`
	2. Create the virtualenv `venv` in the `voice_recognition` directory:
	`virtualenv -p python3 venv`
	3. Activate the virtualenv:
	`source venv/bin/activate`
	4. Install the required packages:
	`pip install pyyaml rospkg rospy std_msgs tensorflow matplotlib`.

	The virtualenv will automatically be activated when running `bin/run.sh` or `bin/check_deps.sh`. On the other hand, `bin/volume_graph.py` requires you to activate the virtualenv manually if matplotlib isn't installed globally.  

## Running
After running `catkin_make`, the package should be ready to launch with `roslaunch voice_recognition main.launch`. Another way to run the package is to just run `bin/run.sh` directly (cd to the bin/ folder first). (I think) this allows you to view more of the log output from the script. If you're just running the script directly, there is no need to call `catkin_make`. Just make sure that roscore is running.

If you're running this script on another machine than roscore, you'll need to manually edit `ROS_MASTER_URI` in `bin/run.sh` and point it to the IP/port of the roscore instance. I have not tried using roslaunch under this configuration, so I recommend manually editing the variable, and launching the script directly.

The script can also be run without ROS installed. If the packages `rospy` or `std_msgs` fail to import in Python, the script will output to the standard out stream. This behaviour can also be triggered by passing `--stdout` to `run.sh`, like so: `./run.sh --stdout`.
