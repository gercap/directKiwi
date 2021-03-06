# directKiwi

GUI version 3.63 for Python 2.7 written by linkz using modified versions @ https://github.com/dev-zzo/kiwiclient or related fork @ https://github.com/jks-prv/kiwiclient

## LICENSE
* This code has been written and released under the "do what the f$ck you want with it" license

## WARNING
* this code may contain some silly procedures and dumb algorithms as I'm not a python guru, but it works so...

## TODO LIST
* enable IQ mode with direct recording to file ?

## CHANGE LOG 
* v1.00 : first working version, basic, only freq/mode/bw, connect/disconnect
* v1.1beta : adding the smeter
* v1.10 : design modification + console output
* v1.20 : adding the spectrum extension
* v1.30 : adding the recorder extension
* v1.31 : dealing with processes & sub-processes + code clean-up
* v1.32 : changed LP & HP from QSlider to QScrollBar style for 100Hz steps clicks + some comments added + code clean-up
* v1.33 : AM modulation has fixed BW (10kHz) - CW has frequency offset -1kHz BW set to 200Hz centered on desired freq
* v1.40beta : The KiwiSDR manual server listing update has been added - note: requires pygeoip module, still some bugs
* v1.40 : most "update proc" bugs has been removed (major concerns wrong user entries in their kiwisdr setup/reg)
* v1.41 : adding the possibility to manually enter IP:PORT/HOSTNAME:PORT to connect to a specific KiwiSDR server
* v1.42 : adding some regexp checks to validate frequencies and IP:PORT/HOSTNAME:PORT in both top inputboxes
* v2.00 : BUG SOLVED: "QScrollbar & TextEdit refresh still not working with direct program lunch"
* v2.01 : KiwiSDRclient.py cleaned + S-meter bars removed
* v2.10 : adding the AGC control (manual or auto)
* v2.20 : spectrum & recording modules has been removed, too many issues with client-side sound card settings
* v2.30 : it's now possible to switch from node to node directly by double-clicking on the listing lines (prev forgiven)
* v3.00 : GUI based on TK now, no more PyQT stuff needed
* v3.10 : adding labels, node listing and console scrollbars, cancel connect possibility + all CRLF converted to LF
* v3.11 : some modifications in the console output log format + an update process bug has been solved
* v3.20 : update process finally re-written and still using GeoIP - node listing now redrawing itself, no more restart needed
* v3.30 : host ports are checked before filling the node listing, normally all nodes listed are reachable now
* v3.40 : adding top bar menus for colors changing and saving directKiwi.cfg configuration (colors + geometry)
* v3.50 : adding default lowpass, highpass, modulation to directKiwi.cfg configuration file
* v3.60 : adding default agc/mgc, mgc gain, hang, agc threshold, agc slope, agc decay to directKiwi.cfg configuration file
* v3.61 : agc/mgc listbox now (previously checkbox) + xxx.proxy.kiwisdr.com hosts (out of USA) locations fixed + autosorting list  at start
* v3.62 : fixed a TK issue than was caused by python 2.7.15 version (code was written under 2.7.14
* v3.63 : fixed an issue with the India located remote that has few informations in kiwisdr.com/public listing page (source for updates)

\\\ USAGE

linux   : ./directKiwi.py
windows : double-click on directKiwi.py
macOS X : ./directKiwi.py ?



\\\ INFO 1  -  You need python 2.7.x to run this script, it will NOT work with python 3.x !

linux   : "sudo apt-get install python2.7" 
 note that python 2.x is probably already installed in your distro (type "python --version" to see which version is)
 in some cases you may also have to manually install python-tk with "sudo apt-get install python-tk"

windows : https://www.python.org/ftp/python/2.7/python-2.7.amd64.msi or https://www.python.org/ftp/python/2.7/python-2.7.msi

macOS X : https://www.python.org/ftp/python/2.7/python-2.7-macosx10.5.dmg or https://www.python.org/ftp/python/2.7/python-2.7-macosx10.3.dmg



\\\ INFO 2  -  This script requires the following python modules : 'pygame' 'numpy' 'pygeoip' ..... (and 'scipy' for MacOS X users)

linux   : "python -m pip install pygame numpy pygeoip" in terminal
 NOTE: for debian/ubuntu: "sudo apt-get install python-pygame python-numpy python-pygeoip" at once should work 
 but python-pygeoip may not install on stable debian distro, use the pip way instead for this module..
 also, if 'pip' is not already installed on your distro : "sudo apt-get install python-pip"

windows : "python -m pip install pygame numpy pygeoip" in 'cmd'

macOS X : "python -m pip install pygame numpy pygeoip scipy" in terminal



Enjoy

linkz
Apr 2018
