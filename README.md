# lookat

Die Software lookat arbeitet als Fotofalle

Sie erkennt Unterschiede zum Vorgängerframe und startet
bei entsprechender Differenz eine kurze Viedeoaufzeichnung.

Die aufgezeichneten Video's werden in Verzeichnis home/lookat_video/Datum abgelegt.

Verwendete Hardware
- Intel® Core™ i7-7500U CPU @ 2.70GHz × 4
- Raspi 4
- Raspi Pi Zero

<pre>
Usage: ./lookat [options] 
Options: 
  -h --help            Print this help screen
  -e --threshold (arg) Schwellwert; default: 64
  -c --cam (arg)       Kamera-Nr; default: 0. Verfügbare Kameras lassen sich mit ls /dev/video* anzeigen.
  -m --manuell         Start/Stop prozess with key 'm'
  -n --noutput         keine Bildschirmausgabe
  -d --diff (arg)      Pixel-Differenz zum Vorgängerbild [1..5000]; default: 5
  -g --gray            Save grayscale
  -a --trail (arg)     Nachlauf in frames; default: 7
  -p --picture         save only picture
  -w --camwidth (arg)  Kamerabild Breite; default: 640
  -i --camheight (arg) Kamerabild Höhe; default: 480

  --pixdiff (arg)      Pixel-Differenz[0..5000] für Mosaik-Segment; default: 25
  --minvidtime (arg)   Min.Videolänge in [ms]; default: 2700 ms. kleinster Wert ist 2000 ms.
  --maxvidtime (arg)   Max.Videolänge in [ms]; default: 20000 ms.
  --maxvideo (arg)     Max.Anzahl Video's; default -1, d.h keine Begrenzung
  --vidpath (arg)      Pfad zum Sichern der Videos; default: ~/lookat_video/DATUM

------ Sensitiver Bildausschnitt ------
  -l --left (arg)       left roi
  -r --right (arg)      right roi
  -t --top (arg)        top roi
  -b --bottom (arg)     bottom roi

------ hot key's ------
      ESC, q = Programende
           h = this message
           m = toogle prozess
           i = show parameter
</pre>
