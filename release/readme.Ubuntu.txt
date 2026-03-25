1) install esptool via pipx
sudo apt update
sudo apt install -y pipx
pipx ensurepath
# reopen terminal (or: source ~/.profile)
pipx install esptool

Now you should have:
esptool.py --help
(you may have to open a new terminal)

2) Find port
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
(mine is: /dev/ttyACM0)

3) Add your user to the right group (usually dialout)
sudo usermod -aG dialout $USER
newgrp dialout

4) Flash (change port according to 2)
chmod +x flash.sh

./flash.sh
