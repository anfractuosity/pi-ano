# pi-ano

play tunes via capacitance with the pi

Connect pear or banana or.. to GPIO 17

```
sudo apt-get install portaudio19-dev
sudo gpasswd -a root pulse-access 
sudo pulseaudio -D --system

make 
sudo ./pi-ano
```
