#! /bin/sh

# ./lookat $@ -m -l 0 -r 600 -t 100 -b 350 --camwidth 640 --camheight 480 --maxvidtime 50000
./lookat $@ -m -l 0 -r 600 -t 100 -b 350 --camwidth 800 --camheight 800 --maxvidtime 50000 --ignorleft -50 --ignortop -50 --ignorwidth 400 --ignorheight 250
# ./lookat $@ -m --camwidth 640 --camheight 480 --maxvidtime 50000
# ./lookat $@ -m -l 50 -r 600 -t 100 -b 350 --camwidth 800 --camheight 600 
