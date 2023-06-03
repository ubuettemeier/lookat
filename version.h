

#define STR2(x)  #x
#define STR(x) STR2(x)

#define VERSION_MAJOR 0
#define VERSION_MINOR 8
#define VERSION_PATCH 7

#define VERSION ("v" STR(VERSION_MAJOR) "." STR(VERSION_MINOR) "." STR(VERSION_PATCH))
// #define VERSION ("v" STR(VERSION_MAJOR) "." STR(VERSION_MINOR))

/*
v0.6.5    struct geo NEW
v0.6.6    Tastensteuerung fuer struct geo angelegt.
v0.7.0    ROI Definition mit Start-Parameter einstellen.
v0.7.1    NonZero Schwelle auf 41,6% von back-screen festgelegt.
v0.7.2    Sensitive Bildfläche im Videobild als rectangle eingefügt.
v0.7.3    Doxygen Optimierungen
v0.7.4    Plausibilitätsprüfung für die Parameter des Sensitiven Bildausschnitts eingebaut.
v0.7.5    Parameter --camwidth --camheight NEW
v0.7.6    Aufräumarbeiten
v0.7.7    Parameter --maxvideo NEW
v0.7.8    Doxygen Optimierungen. Aufräumarbeiten
v0.8.0    Min. und Max. Videolänge in [ms] eingefuehrt.
v0.8.1    Makefile erweitert.
v0.8.2    README.md hinzu.
v0.8.3    Option --vidpath NEW.
v0.8.4    Funktion make_path() NEW.
v0.8.5    #include <filesystem> ausgetauscht gegen <experimental/filesystem>
v0.8.6    run.sh NEW
v0.8.6    make_path() mit __cplusplus Abtrage versehen. Ist für <filesystem> erforderlich.
v0.8.7    github repository created.
*/
