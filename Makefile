.PHONY: start restart run

VENV_DIR := ./IESL-RoboGames-Uni-Phase1/venv
PYTHON := $(VENV_DIR)/bin/python
PIP := $(VENV_DIR)/bin/pip
WEBOTS_HOME := /snap/webots/current/usr/share/webots

get_ardupilot:
	@if [ ! -d ardupilot ]; then \
		git clone --depth 1 --recurse-submodules --shallow-submodules https://github.com/ArduPilot/ardupilot.git; \
	else \
		echo "ardupilot already exists"; \
	fi

sa:
	nice -n -20 $(PYTHON) ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -w --model webots-python --add-param-file=ardupilot/libraries/SITL/examples/Webots_Python/params/iris.parm --out=udp:127.0.0.1:14550


sw:
	webots --mode=realtime ./IESL-RoboGames-Uni-Phase2/Webots/worlds/iris_Task_2.wbt

sc:
	WEBOTS_HOME="/snap/webots/current/usr/share/webots" \
	PYTHONPATH="$(WEBOTS_HOME)/lib/controller/python:$$PYTHONPATH" \
	LD_LIBRARY_PATH="$(WEBOTS_HOME)/lib/controller:$$LD_LIBRARY_PATH" \
	nice -n -20 $(PYTHON) ./IESL-RoboGames-Uni-Phase2/Webots/controller/ardupilot_vehicle_controller.py \
		--motors m1_motor,m2_motor,m3_motor,m4_motor \
		--camera camera \
		--camera-port 5599
		
run:
 	$(PYTHON) ./IESL-RoboGames-Uni-Phase2/Task/flight.py

