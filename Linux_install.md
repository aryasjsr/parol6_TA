# How to install on Linux

These steps are intended for Ubuntu 24.04 and other recent Ubuntu releases.
The project has been tested with Python 3.10.x, and some pinned dependencies
such as `numpy==1.23.4` are not suitable for the system Python 3.12 that ships
with Ubuntu 24.04.

## 1. Install system packages

```bash
sudo apt update
sudo apt install -y git build-essential python3-tk python3-pil python3-pil.imagetk
```

If you will install Python 3.10 with `pyenv`, also install the Python build
dependencies:

```bash
sudo apt install -y make libssl-dev zlib1g-dev libbz2-dev libreadline-dev \
    libsqlite3-dev curl libncursesw5-dev xz-utils tk-dev libxml2-dev \
    libxmlsec1-dev libffi-dev liblzma-dev
```

## 2. Install Python 3.10

Use a Python 3.10 interpreter that is separate from the operating system
Python. One option is `pyenv`:

```bash
curl https://pyenv.run | bash
```

Follow the shell setup instructions printed by `pyenv`, restart the terminal,
then install Python 3.10:

```bash
pyenv install 3.10.14
```

## 3. Create the virtual environment

From the repository root:

```bash
cd /home/arya/TA/parol6_TA
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools
python -m pip install wheel==0.42.0
pip install -r requirements.txt
```

If Python 3.10 was installed with `pyenv`, use:

```bash
~/.pyenv/versions/3.10.14/bin/python -m venv .venv
```

## 4. Run the application

Use the Linux runner:

```bash
cd /home/arya/TA/parol6_TA
./run_linux.sh
```

Or run the entrypoint manually from `GUI/files`:

```bash
cd /home/arya/TA/parol6_TA/GUI/files
../../.venv/bin/python Serial_sender_good_latest.py
```

Running from `GUI/files` keeps local imports and image/program assets on the
expected relative path.

## 5. Serial USB access

Connect the PAROL6 control board and check which serial device was assigned:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

If the board appears as `/dev/ttyACM0`, enter `0` in the GUI port field.
The application will open `/dev/ttyACM0`.

If the board appears as another device, enter the full path, for example:

```text
/dev/ttyUSB0
```

To avoid running `chmod` every time, add your user to the serial device group
and then log out and log back in:

```bash
sudo usermod -aG dialout arya
```

For a temporary test only:

```bash
sudo chmod 666 /dev/ttyACM0
```

## 6. Quick verification

```bash
cd /home/arya/TA/parol6_TA
.venv/bin/python -c "import serial, customtkinter, roboticstoolbox, numpy, PIL"
source .venv/bin/activate && pip check
./run_linux.sh
```

If the GUI opens but the robot does not connect, confirm the USB device path,
power/firmware state, and serial permissions.
