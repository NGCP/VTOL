# install dependencies
pip3 install -r requirements.txt

# add pylint commit hook
git config core.hooksPath hooks

if [ ! -f "./simulator/arducopter" ]
then
    download and compile simulator
    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    ./waf configure
    ./waf configure --board sitl
    ./waf copter
    cd ..
    mkdir simulator
    mv ./ardupilot/build/sitl/bin/arducopter ./simulator/
    rm -rf ardupilot
fi
