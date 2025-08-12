# Set up
ROS should be installed from apt.

run the following:
```
rosdep install -i --from-path cyclone_vision --rosdistro jazzy -y
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

