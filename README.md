# pi-ano

play tunes via capacitance with the pi. pear not included.

Based on idea found here - http://blog.eikeland.se/2015/04/24/banana-piano/

Uses:

* https://www.raspberrypi.org/forums/viewtopic.php?f=29&t=52393 - to disable interrupts and for gpio setup
* https://elinux.org/RPi_GPIO_Code_Samples#Direct_register_access - for IO macros
* http://portaudio.com/docs/v19-doxydocs/paex__sine_8c_source.html - for generating tones

Connect pear or banana or.. to GPIO 17

```
sudo apt-get install portaudio19-dev
sudo gpasswd -a root pulse-access 
sudo pulseaudio -D --system

make 
sudo ./pi-ano
```

# Todo:

* Add more pears
* Use one pear to cleanly exit, to make sure interrupts are enabled on exit

