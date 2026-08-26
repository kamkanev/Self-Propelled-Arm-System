# Jetank Configuration Guide

Step by step walkthrough for the team to connect JETANK to local computer and establish a connection.
---

##  Prerequisites
Before starting, have the following.
* **Hardware:** JETANK, Micro-USB to USB-A cable
* **Software:** [PuTTY](https://www.putty.org/) installed on your computer.


## Step-by-step Setup Instructions

### Step 1: Connect to the Robot via Cable
To establish an initial connection without a monitor attached to the robot, you must bridge a direct serial interface via a USB cable.

1. Connect the Micro-USB end directly into the Jetson Nano.

### Step 2: Establish a Serial Session Using PuTTY
1. Open **Device Manager** on Windows to identify which port your robot is using.
2. Launch **PuTTY**.
3. Configure the following session settings:
   * **Connection type:** `Serial`
   * **Serial line:** Enter your identified port (e.g., `COM3`)
4. Click **Open**. A blank terminal prompt will appear.
5. Press **Enter** on your keyboard to trigger the login interface. 
6. Log in using your username and password `jetbot`

### Step 3: Connect the Robot to Wi-Fi
Once logged into the shell terminal, use the NetworkManager command-line interface (`nmcli`) to scan for and connect to your local wireless internet infrastructure.

1. **Scan for available Wi-Fi networks to see if wifi adapter works.:**
   ```bash
   nmcli device wifi list
   ```

   To connect to wifi use the following command :
   ```bash 
   sudo nmcli device wifi connect "YOUR_SSID" password "YOUR_PASSWORD"
   ```

    After establishing connection. Connecting to the robot works by getting it's ip and opening the browser and typing it and putting :8888 at the end.

To find it's wlan0 address, use the following bash command
```bash 
ifconfig wlan0
```



   