This firmware can work with both wired bandanna and Bluetooth bandanna.
Attention !!!!
If you only use a wired bandanna solder 10K ~ 100K between input State and GND, therwise anti-cheater will not work correctly.

If you do not want to solder a resistor, then in the file isr.c correct this

if((!(TSOP_IN&TSOP_PIN))&&(!(BT_STATE_IN&BT_STATE_PIN)))
{
if (chit_detected_counter < (4000)) chit_detected_counter++;
if (chit_detected_counter >= (4000)) chit_detected=true;
}

should look like this

if((!(TSOP_IN&TSOP_PIN))/*&&(!(BT_STATE_IN&BT_STATE_PIN))*/)
{
if (chit_detected_counter < (4000)) chit_detected_counter++;
if (chit_detected_counter >= (4000)) chit_detected=true;
}

or like this

if(!(TSOP_IN&TSOP_PIN))
{
if (chit_detected_counter < (4000)) chit_detected_counter++;
if (chit_detected_counter >= (4000)) chit_detected=true;
}

Rebuild project.


Implemented a variety of carrier frequency IR signal through the settings.

Program to configure the "Ascetic" and upload sounds.

NEW!!!!

http://sourceforge.net/projects/ascetconfigurat/files/Ascet-configurator/Qt5_version/02_10_2014/Configurator_and_libs_10_sounds.zip/download


PonyProg
serial device programmer

http://www.lancos.com/prog.html

PC software to use bootloader

http://download.chip45.com/chip45boot2_GUI_V1.13.zip
