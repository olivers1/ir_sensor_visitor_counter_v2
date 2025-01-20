# ir_sensor_visitor_counter_v2
a visitor counter to detect entrance/exit by calculating mean value of the timestamps, for each of the IR-sensors, that are generated when the sensor is being blocked


==SETUP INSTRUCTIONS==
--description--
instructions on how to install and setup Raspi OS in the Raspberry Pi, to setup network connection, SSH, to clone repository from github and install required python packages

--raspberry_pi_setup--
-install raspios on an SD-card
https://www.cyberciti.biz/faq/how-to-write-raspbian-jessie-image-file-to-sd-cards-on-apple-macos-os-x/
download raspbian
open a terminal window run:
unzip 2017-06-21-raspbian-jessie.zip
#locate name of SD-card (usually named disk2)
diskutil list
#unmount SD-card
diskutil unmountDisk /dev/disk2
#write to SD-card
sudo dd bs=1m if=2017-06-21-raspbian-jessie.img of=/dev/rdisk2
#wait until finished
#unmount SD-card
diskutil eject /dev/rdisk2

insert SD-card in raspberry pi

-configuration
in terminal run:
sudo raspi-config
select 'Interface Options'
SSH -> enable
SPI -> enable

-change keyboard layout
in GUI click:
Preferences/'Keyboard and Mouse'/Keyboar/'Keyboard Layout'

-set static IP-address
in terminal run:
sudo nano /etc/dhcpcd.conf

change fields:
interface NETWORK
static ip_address=STATIC_IP/24
static routers=ROUTER_IP
static domain_name_servers=DNS_IP

example: interface wlan0
static ip_address=192.168.1.120/24
static routers=192.168.1.254
static domain_name_servers=192.168.1.254

#for changes to take effect
sudo reboot

-ssh authentification
generate ssh key according to below format
in terminal window run
(mkdir ~/.ssh folder)
(cd ~/.ssh)
ssh-keygen -t rsa -b 4096

#don't provide any the key any name nor password
cat id_rsa.pub
add ssh key .pub to github

--git setup
-clone
git clone git@github.com:olivers1/ir_sensor_visitor_counter.git

-git push setup
git config --global user.email "you@example.com"
git config --global user.name "Your Name"

add ssh key to client laptop to add to github account and for setting up smooth raspberry pi access without password scp olivers@192.168.0.10:~/.ssh/id_rsa.pub ~/Downloads

#if problem with authentification locate 'known_hosts' file and delete same ip adress key in ~/.ssh/known_hosts on local laptop (client)
-required python packages
#install all packages from file

pip install -r requirements.txt

pip install Adafruit-GPIO
pip install Adafruit-MCP3008
pip install spidev
