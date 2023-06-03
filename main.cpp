/*! ------------------------------------------
 * @defgroup main Main: Hauptprogramm
 * @{
 * 
 * @brief Fotofalle
 * @file main.cpp
 * @author Ulrich Buettemeier
 * @date 2021-11-01
 * 
 * @copyright Copyright (c) 2021, 2022, 2023 Ulrich Buettemeier, Stemwede
 * 
 * @todo    - Bildmodus optimieren !!!
 *          - System auf Raspi zero 2 w portieren.
 *          - Programm beenden mit SIGNAL. S.auch killall
 * 
 * @note <guvcview> funktioniert nicht mit der raspi-camera!
 */

/*! -----------------------------------------------------------------------------------------
 * @mainpage Das Projekt lookat
 *
 * Die Software lookat reagiert auf Veränderungen im Videobild. \n
 * Sie erkennt Unterschiede zum Vorgängerframe und startet  \n
 * bei entsprechender Differenz eine kurze Viedeoaufzeichnung. \n
 * \n
 * Die aufgezeichneten Video's werden in Verzeichnis home/lookat_video/<Datum> abgelegt. \n
 * \n
 * Verwendete Hardware \n
 * - Intel® Core™ i7-7500U CPU @ 2.70GHz × 4 \n
 * - Raspi 4 \n
 * - Raspi Pi Zero \n
 * \n
Usage: ./lookat [options]\n
Options:\n
  -h --help            Print this help screen\n
  -e --threshold <arg> Schwellwert; default: 64\n
  -c --cam <arg>       Camera-Nr; default: 0\n
  -m --manuell         Start/Stop prozess with key 'm'\n
  -n --noutput         keine Bildschirmausgabe\n
  -d --diff <arg>      Pixel-Differenz zum Vorgängerbild [1..5000]; default: 5\n
  -g --gray            Save grayscale\n
  -a --trail <arg>     Nachlauf in frames; default: 7\n
  -p --picture         save only picture\n
  -w --camwidth <arg>  Kamerabild Breite; default: 640\n
  -i --camheight <arg> Kamerabild Höhe; default: 480\n
\n
  --pixdiff <arg>      Pixel-Differenz[0..5000] für Mosaik-Segment; default: 25\n
  --minvidtime <arg>   Min.Videolänge in [ms]; default: 2700 ms. kleinster Wert ist 2000 ms.\n
  --miaxidtime <arg>   Max.Videolänge in [ms]; default: 20000 ms.\n
  --maxvideo <arg>     Max.Anzahl Video's; default -1, d.h keine Begrenzung\n
  --vidpath <arg>      Pfad zum Sichern der Bewegungs-Videos; default: ~/lookat_video/DATUM\n
\n
------ Sensitiver Bildausschnitt ------\n
  -l --left <arg>       left roi\n
  -r --right <arg>      right roi\n
  -t --top <arg>        top roi\n
  -b --bottom <arg>     bottom roi\n
\n
------ hot key's ------\n
      ESC, q = Programende\n
           h = this message\n
           m = toogle prozess\n
           i = show parameter\n
\n
@code
./lookat --camwidth 800 --camheight 600 -l 50 -r 750 -t 150 -b 400 --maxvideo 50
@endcode
 */

// -------- define for developer -------
#define USE_HARRIS_DETECTOR_        //!< Versuch: Über die Anzahl der Punkte eine Bewegung erkennen.
#define USE_FEATURE_DETECTOR_

#ifdef USE_FEATURE_DETECTOR
    #define USE_HARRIS_DETECTOR
#endif

#define SHOW_MOSAIK_                //!< SHOW_MOSAIK zeigt ein screen mit dem Mosiak.
#define SHOW_HISTOGRAM_             //!< SHOW_HISTOGRAM zeigt das original und gestretchte Histogram.
// -------------------------------------
#define RASPI_

#include <iostream>
#include <termios.h>
#include <getopt.h>
#include <fstream>
#if __cplusplus >= 201703L
    #include <filesystem>                // Offiziele stabile Version. Ab c++17 möglich
#else
    #include <experimental/filesystem>      // experimentelle Version. Wird nicht mehr weiter entwickelt
#endif

#include <cstdint>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <pwd.h>
#include <fcntl.h>
#include <unistd.h>
#include <cassert>
#include "timefunc.hpp"
#include "version.h"

#ifdef RASPI
    #include "raspi.hpp"
#endif

#include "Save_Vid.hpp"
#include "histogram.h"
#ifdef USE_HARRIS_DETECTOR
    #include "harrisDetector.h"
#endif

#include "opencv2/opencv.hpp"

#define USE_CVD_

#ifdef USE_CVD
    #include "../../OpenCVD/include/opencvd.hpp"
    #include "../../OpenCVD/include/specdef.hpp"
#else 
    #define CVD cv
#endif

using namespace std;
using namespace cv;

#define MAX_IN 3
#define HORZ_TEILER 8
#define VERT_TEILER 6
#define MAX_DELAY 100000       //!< Verweilzeit in [us] für @ref get_frame().

cv::Mat in[MAX_IN];
cv::Mat src[MAX_IN];
cv::Mat src_image;

cv::Mat seg[MAX_IN][HORZ_TEILER][VERT_TEILER];              //!< Mosaik-Bild z.B.:  640 x 480 => 80 x 80
cv::Mat seg_diff[HORZ_TEILER][VERT_TEILER];                 //!< Mosaik Differenzbild
cv::Mat seg_NonZero (VERT_TEILER, HORZ_TEILER, CV_8UC1);    //!< Mosaik-Matrix
cv::Mat contours_pic;                                       //!< Contour-Bild
int camwidth = 640;                                         //!< Defaultwert für Parameter --camwidth.
int camheight = 480;                                        //!< Defaultwert für Parameter --camheight.
int contour_x_center = 0;                                   //!< Konturschwerpunkt in X
#ifdef SHOW_MOSAIK
    cv::Mat show_seg;                           // Ausgabebild für Mosaik
#endif

cv::Mat diff, back;
uint8_t first_in = 0, last_in = MAX_IN-1;   //!< Ringzähler
int anz_zero[MAX_IN] = {-1};                //!< Speicher für NonZero-Werte
time_t now[MAX_IN];                         //!< Aufnahme-Zeitpunkt

cv::VideoCapture cap;           //!< Kamera Konstructor. Gestartet wird die Kamera mit <cap.open()>
save_video sv;                  //!< class {@ref Save_Vid.hpp} initialisieren

#ifdef USE_HARRIS_DETECTOR
    cv::Mat harrisCorners;
    HarrisDetector harris;
#endif

#ifdef USE_FEATURE_DETECTOR
    // cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(0);
    cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(25, false, FastFeatureDetector::TYPE_7_12);
    cv::Ptr<cv::DescriptorExtractor> descriptor;
    std::vector<cv::KeyPoint> pyrKeypoints;
#endif

uint16_t state = 0;             //!< wird in @ref control() verwendet
int vid_counter = 0;
std::string folder;             //!< Ausgabeverzeichnis; default: ~/lookat_video/DATUM  kann mit der Option --vidpath eingestellt werden.
std::string home_dir;           //!< Home Verzeichnis @see {@ref get_homedir()}

#pragma pack(1)

struct _properties_ {
    int cam_index = 0;          //!< Kann mit Parameter --cam geändert werden.
    int threshold = 64;         //!< Alle Pixel in diff unter 64 werden auf 0 gesetzt. @see @ref schwelle()
    int diff_non_zero = 0;      //!< Enthält die aktuelle NonZero Differenz von last_in - first_in. 
    int video_start_diff = 5;   //!< sobald @ref diff_non_zero >= video_start_diff ist, wird eine Aufnahme gestartet! Wertebereich: [1...5000]. @see @ref control()
    bool falle_aktiv = false;   //!< Flag zeigt an, ob eine Bewegung erkannt wurde. @ref get_frame().
    int trail = 7;              //!< Nachlauf in frames. ca.1200 ms
    bool run = true;
    bool no_output = false;     //!< bei true wird kein Camerabild gezeigt. Wird mit der Option --noutput gesetzt.
    int frame_delay = MAX_DELAY;    //!< Verweilzeit in [us] für @ref get_frame().
    int NonZero_seg = 25;       //!< pixdiff für Mosaik. Lässt sich mit der Option --pixdiff ändern.
    int min_time = 2700;        //!< Min.Videolänge in [ms]. Kleinster zulässiger Wert ist 2000 ms
    int max_time = 20000;       //!< Max.Videolänge in [ms].
    bool only_picture = false;  //!< Bei true werden nur Bilder gespeichert, kein Videos. Wird mit der Option --picture eingeschaltet.
    std::string vidpath;
} properties;

/*! ----------------------------------------------------------------------
 * @brief Sensitiver Bildausschnitt
 */
struct _geo_ {
    int left = 0;
    int top = 0;
    int right = 639;        // if (fwidth > geo.width) geo.width = fwidth;
    int bottom = 479;       // if (fheight > geo.height) geo.height = fheight;
} geo, new_geo = {-1, -1, -1, -1};

#pragma pack()

 // Variablen werden für kbhit() und getch() benötigt
static struct termios initial_settings, new_settings;   //!< globale value für {@ref kbhit()} und {@ref getch()}
static int peek_character = -1;                         //!< globale value für {@ref kbhit()} und {@ref getch()}

// ------------------ Camera Parameter ----------------------------
double saturation;      //!< Sättigung
double brightness;      //!< Helligkeit
double contrast;        //!< Kontrast
double exposure;        //!< Belichtungswert
double fwidth;          //!< Kamera Bildbreite
double fheight;         //!< Kamera Bildhöhe

// --------------------- Prototypen ----------------------------------
static void help ();
static void show_short_keys ();
static void show_cplus_version ();
void check_long_options (struct option *opt, char *optarg);
int check_plausibiliti_of_opt ();
int control_opt (int argc, char ** argv);
void show_properties ();

static void show_geo ();
static void reset_geo ();

void init_keyboard ();
void close_keyboard ();
int getch ();
int kbhit ();

inline bool file_exists (const std::string& name);
void init_vid_counter ();
int make_path (std::string pname);
int init_folder ();

cv::Mat make_ausgabe_screen (cv::Mat src, cv::Mat seg_screen);
void make_seg (cv::Mat src);
void write_diff_non_zero_to_diff ();
void schwelle (cv::Mat &s, uint8_t schwellwert);    // Schwellwertfunktion für Differenz-bild
void get_frame ();

static void control ();
static void get_cam_para ();
static void show_cam_para ();

/*! --------------------------------------------------------------
 * @brief Ausgabe der Kameraparameter.
 */
void show_properties ()
{
    cout << "-------- properties ---------\n";
    cout << "--threshold   " << properties.threshold << endl;
    cout << "--cam         " << properties.cam_index << endl;
    cout << "--diff        " << properties.video_start_diff << endl;
    cout << "--trail       " << properties.trail << endl;
    cout << "--pixdiff     " << properties.NonZero_seg << endl;
    cout << "--minvidtime  " << properties.min_time << " ms\n";
    cout << "--maxvidtime  " << properties.max_time << " ms\n";
    cout << "--vidpath     " << properties.vidpath << endl;
    cout << "--frame_delay " << properties.frame_delay/1000 << " ms\n";
}

/*! --------------------------------------------------------------
 * @brief Ausgabe der Parameterliste.
 */
static void help()
{
    cout << "\n";
    cout << "lookat " << VERSION << endl;
    cout << "(C) 2021 Ulrich Buettemeier\n\n";
    cout << "Usage: ./lookat [options]\n";
    cout << "Options:\n";
    cout << "  -h --help            Print this help screen\n";
    cout << "  -e --threshold <arg> Schwellwert; default: " << properties.threshold << endl;
    cout << "  -c --cam <arg>       Camera-Nr; default: " << properties.cam_index << endl;
    cout << "  -m --manuell         Start/Stop prozess with key 'm'\n";
    cout << "  -n --noutput         keine Bildschirmausgabe\n";
    cout << "  -d --diff <arg>      Pixel-Differenz zum Vorgängerbild [1..5000]; default: " << properties.video_start_diff << endl;
    cout << "  -g --gray            Save grayscale\n";
    cout << "  -a --trail <arg>     Nachlauf in frames; default: " << properties.trail << endl;
    cout << "  -p --picture         save only picture\n";
    cout << "  -w --camwidth <arg>  Kamerabild Breite; default: 640\n";
    cout << "  -i --camheight <arg> Kamerabild Höhe; default: 480\n";
    cout << endl;
    cout << "  --pixdiff <arg>      Pixel-Differenz[0..5000] für Mosaik-Segment; default: " << properties.NonZero_seg << endl;
    cout << "  --minvidtime <arg>   Min.Videolänge in [ms]; default: 2700 ms. Kleinster Wert ist 2000 ms\n";
    cout << "  --miaxidtime <arg>   Max.Videolänge in [ms]; default: 20000 ms\n";
    cout << "  --maxvideo <arg>     Max.Anzahl Video's; default -1, d.h keine Begrenzung\n";
    cout << "  --vidpath <arg>      Pfad zum Sichern der Bewegungs-Videos; default: ~/lookat_video/DATUM\n";
    cout << endl;
    cout << "------ Sensitiver Bildausschnitt ------\n";
    cout << "  -l --left <arg>      left roi\n";
    cout << "  -r --right <arg>     right roi\n";
    cout << "  -t --top <arg>       top roi\n";
    cout << "  -b --bottom <arg>    bottom roi\n";
    cout << endl;
    show_short_keys ();
}

/*! ----------------------------------------
 * @brief Ausgabe der Tastenbelegung
 */
static void show_short_keys ()
{
    cout << "------ hot key's ------\n";
    cout << "      ESC, q = Programende\n";
    cout << "           h = this message\n";
    cout << "           m = toogle prozess\n";
    cout << "           i = show parameter\n";
    cout << "           f = show Fileliste\n";
    cout << endl;
}

static void show_cplus_version ()
{
    if (__cplusplus == 201703L) {
        std::cout << "C++17 wird unterstützt." << std::endl;
    } else if (__cplusplus == 201402L) {
        std::cout << "C++14 wird unterstützt." << std::endl;
    } else if (__cplusplus == 201103L) {
        std::cout << "C++11 wird unterstützt." << std::endl;
    } else {
        std::cout << "C++-Version wird nicht erkannt." << std::endl;
    }
}

/*! ----------------------------------------
 * @brief Ausgabe des sensetiven Bildausschnitts.
 */
static void show_geo ()
{
    cout << "------- sensitiver Bildausschnitt ------\n";
    cout << "  left = " << geo.left << endl;
    cout << "   top = " << geo.top << endl;
    cout << " right = " << geo.right << endl;
    cout << "bottom = " << geo.bottom << endl;
}

/*! ----------------------------------------
 * @brief {@ref struct _geo_} wird mit Screen-Abmessung belegt.\n
 *        Es ist wichtig, das im Vorfeld die Funktion {@ref get_cam_para()} aufgerufen wurde!
 */
static void reset_geo ()
{
    geo.left = (new_geo.left >= 0) ? new_geo.left : 0;
    geo.top = (new_geo.top !=-1) ? new_geo.top : 0;
    geo.right = (new_geo.right !=-1) ? new_geo.right : fwidth-1;
    geo.bottom = (new_geo.bottom !=-1) ? new_geo.bottom : fheight-1;

    // Plausibilitätsprüfung
    if ((geo.left > fwidth-1) || (geo.left >= geo.right)) {
        cout << "ERROR Parameter --left out of range. --left wird auf 0 gesetzt\n";
        geo.left = 0;
    }
    if ((geo.right > fwidth-1) || (geo.right <= geo.left)) {
        cout << "ERROR Parameter --right out of range. --right wird auf MAX gesetzt\n";
        geo.right = fwidth - 1;
    }
    if ((geo.top > fheight-1) || (geo.top >= geo.bottom)) {
        cout << "ERROR Parameter --top out of range. --top wird auf 0 gesetzt\n";
        geo.top = 0;
    }
    if ((geo.bottom > fheight-1) || (geo.bottom <= geo.top)) {
        cout << "ERROR Parameter --bottom out of range. --bottom wird auf MAX gesetzt\n";
        geo.bottom = fheight-1;
    }
}

/*! --------------------------------------------------------------
 * @brief es werden die Option --pixdiff und --maxframe ausgewertet
 */
void check_long_options (struct option *opt, char *optarg)
{
    if (strcmp (opt->name, "pixdiff") == 0) {           // option --pixdiff
        if (opt->has_arg == required_argument) {
            int foo;
            try {
                foo = std::stoi (optarg);
            } catch (std::invalid_argument const& ex) {
                std::cout << "--pixdiff ERROR " << "#1: " << ex.what() << '\n';
                return;
            }
            if ((foo >= 0) && (foo <= 5000)) {
                properties.NonZero_seg = foo;
                cout << "pixdeiff = " << properties.NonZero_seg << endl;
            } else 
                cout << "ERROR: falscher Parameter für pixdiff [0..5000]\n";
        } else
            cout << "wrong parameter for optin --pixdiff\n";
    // ---------------------- min videotime --------------------------------
    } else if (strcmp (opt->name, "minvidtime") == 0) {           // option --minvideotime
        if (opt->has_arg == required_argument) {
            int foo;
            try {
                foo = std::stoi (optarg);
            } catch (std::invalid_argument const& ex) {
                std::cout << "--minvidtime ERROR " << "#1: " << ex.what() << '\n';
                return;
            }
            if ((foo >= 2000)) {          
                properties.min_time = foo;
                cout << "minvidtime = " << properties.min_time << endl;
            } else {
                cout << "ERROR: falscher Parameter für minvidtime [>=2000 ms]\n";
                cout << "minvidtime wird auf 2000 ms gesetzt\n";
                properties.min_time = 2000;
            }
        } else
            cout << "wrong parameter for optin --mnvidtime\n";
    // ---------------------- max videotime --------------------------------
    } else if (strcmp (opt->name, "maxvidtime") == 0) {           // option --minvideotime
        if (opt->has_arg == required_argument) {
            int foo;
            try {
                foo = std::stoi (optarg);
            } catch (std::invalid_argument const& ex) {
                std::cout << "--maxvidtime ERROR " << "#1: " << ex.what() << '\n';
                return;
            }
            properties.max_time = foo;
            cout << "maxvidtime = " << properties.max_time << endl;
        } else
            cout << "wrong parameter for optin --maxidtime\n";
    // ------------------------- max video --------------------------------------
    } else if (strcmp (opt->name, "maxvideo") == 0) {           // option --maxvideo
        if (opt->has_arg == required_argument) {
            int foo;
            try {
                foo = std::stoi (optarg);
            } catch (std::invalid_argument const& ex) {
                std::cout << "--maxvideo ERROR " << "#1: " << ex.what() << '\n';
                return;
            }
            sv.set_maxvideo (foo);
        } else
            cout << "wrong parameter for optin --maxvid\n";
    // ---------------------
    } else if (strcmp (opt->name, "vidpath") == 0) {           // option --maxvideo
        if (opt->has_arg == required_argument) {
            cout << "Path: " << optarg << endl;
            properties.vidpath = optarg;
        } else
            cout << "wrong parameter for optin --vidpath\n";
    // ---------------------            
    } else {
        cout << opt->name << ": unbekannte Option\n";
    }
}

/*! --------------------------------------------------------------
 * \brief Überprüft die Plausibilität der Optionen.
 *
 * Diese Funktion überprüft, ob die maximale Zeit (max_time) kleiner als die minimale Zeit (min_time) ist.
 * Wenn dies der Fall ist, wird eine Warnung ausgegeben und die maximale Zeit auf die minimale Zeit gesetzt.
 *
 * \return immer EXIT_SUCCESS
 */
int check_plausibiliti_of_opt ()
{
    if (properties.max_time < properties.min_time) {
        cout << "WARNING: maxvideotime < minvideotime\n";
        properties.max_time = properties.min_time;
    }
    return EXIT_SUCCESS;
}

/*! --------------------------------------------------------------
 * @brief Parameterliste auswerten.
 * @return immer 0
 */
int control_opt (int argc, char ** argv)
{
    // long para, has_arg, flag, val(short para)
    static const struct option long_options[] =
    {
        { "help", no_argument, 0, 'h' },
        { "threshold", required_argument, 0, 'e' },     // Empfinlichkeit
        { "cam", required_argument, 0, 'c' },           // Camera Index
        { "manuell", no_argument, 0, 'm' },             // manueller Start
        { "noutput", no_argument, 0, 'n' },             // kein Bildschirmausgabe
        { "diff", required_argument, 0, 'd' },          // Pixeldifferenz zum Vorgängerbild <properties.video_start_diff>
        { "gray", no_argument, 0, 'g' },
        { "trail", required_argument, 0, 'a' },         // Nachlauf in frames
        { "picture", no_argument, 0, 'p' },             // Bildmodus
        { "pixdiff", required_argument, 0, 0 },            // Pixel-Differenz[0..5000] für Mosaik-Segment
        { "maxframe", required_argument, 0, 0 },           // Max.Anzahl Bilder pro Video. Default 100
        { "maxvideo", required_argument, 0, 0 },           // Max.Anzahl Video's, default -1, d.h keine Begrenzung
        { "minvidtime", required_argument, 0, 0 },
        { "maxvidtime", required_argument, 0, 0 },
        { "vidpath", required_argument, 0, 0 },
        { "camwidth", required_argument, 0, 'w' },      // Karabild Breite
        { "camheight", required_argument, 0, 'i' },     // Kamerabild Höhe

        // Sensitiver Bildbereich
        { "left", required_argument, 0, 'l' },          
        { "top", required_argument, 0, 't' },
        { "right", required_argument, 0, 'r' },
        { "bottom", required_argument, 0, 'b' },
    };

    while (1) {
        int index = -1;
        struct option * opt = 0;

        int result = getopt_long(argc, argv,
                                 "he:c:mnd:ga:p00l:t:r:b:w:i:",     // h = no para; s: = with para; p: = with para
                                 // "hs:l:mc:nd:t:gr:b0",     // h = no para; s: = with para; p: = with para
                                 long_options, &index);

        if (result == -1) 
            break;          // end of list

        switch (result)
        {
            case 'h':       // help anzeigen, dann stop.
                help();
                exit (0);
                break;
            case 'l': {
                int foo = std::stoi (optarg);
                if ((foo >= 0) && (foo <= 4048)) {
                    new_geo.left = foo;
                    cout << "left=" << new_geo.left << endl;
                } else 
                    cout << "ERROR: falscher Parameter für --left [0..4048]\n";
                }   
                break;
            case 'b': {
                int foo = std::stoi (optarg);
                if ((foo >= 0) && (foo <= 4048)) {
                    new_geo.bottom = foo;
                    cout << "bottom=" << new_geo.bottom << endl;
                } else 
                    cout << "ERROR: falscher Parameter für --bottom [0..4048]\n";
                }   
                break;
            case 't': {
                int foo = std::stoi (optarg);
                if ((foo >= 0) && (foo <= 4048)) {
                    new_geo.top = foo;
                    cout << "top=" << new_geo.top << endl;
                } else 
                    cout << "ERROR: falscher Parameter für --top [0..4048]\n";
                }   
                break;
            case 'w': {     // camwidth
                    int foo = std::stoi (optarg);
                    camwidth = foo;
                    cout << "camwidth: " << camwidth << endl;
                }
                break;
            case 'i': {     // camheight
                    int foo = std::stoi (optarg);
                    camheight = foo;
                    cout << "camheight: " << camheight << endl;
                }
                break;
            case 'r': {
                int foo = std::stoi (optarg);
                if ((foo >= 0) && (foo <= 4048)) {
                    new_geo.right = foo;
                    cout << "right=" << new_geo.right << endl;
                } else 
                    cout << "ERROR: falscher Parameter für --right [0..4048]\n";
                }   
                break;
            case 'p':       // -p --picture   save only picture
                properties.only_picture = true;
                cout << "only picture\n";
                break;
            case 'd': {     // -d --diff <arg>      Pixel-Differenz zum Vorgängerbild [1..5000]; default: " << properties.video_start_diff << endl;
                    int foo = std::stoi (optarg);
                    if ((foo > 0) && (foo <= 5000))
                        properties.video_start_diff = foo;
                    else 
                        cout << "ERROR: falscher Parameter für --diff [1..5000]\n";
                }
                break;
            case 'a': {     // -a --trail <arg>     Nachlauf in frames; default: " << properties.trail << endl;
                    int foo = std::stoi (optarg);
                    properties.trail = foo;
                    cout << "Nachlauf: " << properties.trail << " frames\n";
                }
                break;
            case 'g':       // gray
                sv.set_gray (true);
                break;
            case 'e': {     // -e --threshold <arg> Schwellwert     
                    uint8_t foo = std::stof (optarg);
                    properties.threshold = foo;
                    cout << "sense= " << properties.threshold << endl;
                }
                break;
            case 'c': {     // Camera-Nr
                    int foo = std::stoi (optarg);
                    properties.cam_index = foo;
                }
                break;
            case 'm':       // start prog manuell
                cout << "Start prozess with key m\n";
                properties.run = false;
                break;
            case 'n':       // keine Ausgabe
                properties.no_output = true;
                break;
            case 0: /* all parameter that do not */
                    /* appear in the optstring */
                opt = (struct option *)&(long_options[index]);
                check_long_options (opt, optarg);
                /*
                printf("'%s' was specified.", opt->name);
                if (opt->has_arg == required_argument)
                    printf("Arg: <%s>", optarg);
                printf("\n");
                */
                break;
            default: /* unknown */
                break;
        }
    }

    return 0;
}

/*!	--------------------------------------------------------------------
 *	@brief	Fuer die Nutzung der Funktion kbhit()\n
 * 			ist der Aufruf init_keyboard() erforderlich.\n
 *          Deinstalliert wird das keyboard mit close_keyboard().
 */
void init_keyboard()
{
    tcgetattr(0, &initial_settings);    // settings merken
    new_settings = initial_settings;    // new_settings vorkonfigurieren
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);   // new_settings speichern
}

/*! --------------------------------------------------------------
 * @brief Funktion deinstaliert die Tastenabfrage.\n
 *        kbhit() ist anschliessend nicht mehr nutzbar.
 */
void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);   // settings zurück schreiben
}

/*! --------------------------------------------------------------
 * @brief Funktion prueft, ob eine Taste gedrueckt wurde.
 * @return 0: keine Taste gedrueckt \n
 *         1: Taste gedrueckt. \n\n
 * @code
 *      // Beispiel
 *      if(kbhit()) {
 *          int taste = getch();
 *              switch (taste) {
 *              .
 *              .
 *          }
 *      }
 * @endcode
 */
int kbhit()
{
	unsigned char ch;
	int nread;

    if (peek_character != -1) 
        return 1;
    
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1) {
        peek_character = ch;
        return 1;
    }
    return 0;
}

/*! --------------------------------------------------------------
 * @brief Funktion ermittelt die gedrückte Taste
 * @return Tasten ASCII-Wert
 */
int getch()
{
	char ch;

    if(peek_character != -1) {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0, &ch,1);
    return ch;
}

/*! --------------------------------------------------------------
 * @brief Prüft ob Datei vorhanden ist.
 * @return 1: Datei vorhanden\n
 *         0: keine Datei gefunden
 */
inline bool file_exists (const std::string& fname) 
{
    ifstream f(fname.c_str());  // input file streams
    return f.good();
}

/*! --------------------------------------------------------------------
 * @brief   Funktion sucht die höchste Tages-Video-Nr, z.B out12.avi\n
 *          Es wird dann die nächste Nr als aktueller vid_counter definiert.
 */
void init_vid_counter()
{
    int n=0;
    bool treffer = false;
    char buf[512];

    while ((n<1000) && !treffer) {
        if (!properties.only_picture)
            sprintf (buf, "%s/out%i.avi", folder.c_str(), n);   // video-mode
        else 
            sprintf (buf, "%s/out%i.jpg", folder.c_str(), n);   // picture-mode

        if (file_exists(buf)) 
            ++n;
        else 
            treffer = true;
    }

    if (treffer)
       vid_counter = n; 
}

/*! ----------------------------------------------------------
 * @brief Get the homedir object\n
 *        Pfad wird in {@ref home_dir} abgelegt!
 */
void get_homedir()
{
    uid_t uid = getuid();
    struct passwd *pw = getpwuid(uid);

    if (pw == NULL) {
            printf("get_homedir() Failed\n");
            home_dir = "";
            exit(EXIT_FAILURE);
    }

    home_dir = pw->pw_dir;
}

/*! -----------------------------------------------------------
 * @brief Erstellt ein Verzeichnis mit dem angegebenen Pfad.
 * @param pname Der Pfad des zu erstellenden Verzeichnisses.
 * @return EXIT_SUCCESS, wenn das Verzeichnis erfolgreich erstellt wurde, andernfalls EXIT_FAILURE.
 * @note std::filesystem ist erst mit C++17 möglich !
 */
int make_path (std::string pname)
{
    int ret = EXIT_SUCCESS;

#if __cplusplus >= 201703L      // C++17
    namespace fs = std::filesystem;    // use namespace "experimental::"
    fs::path dummy = pname;

    try {
        if (fs::create_directories(dummy)) {
            // Verzeichnis erfolgreich erstellt
            cout << "--vidpath eingerichtet !\n";   
        } else {
            // Das Verzeichnis existiert bereits oder es gab einen Fehler
            cout << "--vidpath ist schon vorhanden !\n";
        }
        cout << "Path: " << dummy << endl;
    } catch (const std::filesystem::filesystem_error& ex) {
        // Fehler bei der Verzeichniserstellung
        cout << "--vidpath konnte nicht erstellt werden !!! \n";
        cout << "Path: " << dummy << endl;
        ret = EXIT_FAILURE;
    }
#else 
    namespace fs = experimental::filesystem;    // use namespace "experimental::"
    fs::path dummy = pname;

    try {
        if (fs::create_directories(dummy)) {
            // Verzeichnis erfolgreich erstellt
            cout << "--vidpath eingerichtet !\n";   
        } else {
            // Das Verzeichnis existiert bereits oder es gab einen Fehler
            cout << "--vidpath ist schon vorhanden !\n";
        }
        cout << "Path: " << dummy << endl;
    } catch (const experimental::filesystem::filesystem_error& ex) {
        // Fehler bei der Verzeichniserstellung
        cout << "--vidpath konnte nicht erstellt werden !!! \n";
        cout << "Path: " << dummy << endl;
        ret = EXIT_FAILURE;
    }
#endif

    return ret;
}

/*! -------------------------------------------------------------
 * @brief Initialisiert das Verzeichnis @ref folder
 * @return EXIT_SUCCESS, wenn das Verzeichnis erfolgreich initialisiert wurde, andernfalls EXIT_FAILURE.
 */
int init_folder()
{
    struct tm now;
    std::string parent_folder;
    int path_is_ok = EXIT_FAILURE;
    
    time_t t = time(NULL);      // Datum ermitteln
    localtime_r (&t, &now);     // Datum formatieren
    char buf[256];
    sprintf (buf, "/%i_%i_%i", now.tm_mday, now.tm_mon+1, now.tm_year+1900);

    if (!properties.vidpath.empty()) {
        parent_folder = properties.vidpath;
        parent_folder += buf;
        path_is_ok = make_path (parent_folder);
    } 

    if (path_is_ok == EXIT_FAILURE) {
        parent_folder = home_dir;
        parent_folder += "/lookat_video";
        parent_folder += buf;
        if (make_path (parent_folder) == EXIT_FAILURE)
            return EXIT_FAILURE;
    }

    folder = parent_folder;

    return EXIT_SUCCESS;
}

/*! -----------------------------------------------------------
 * @brief 
 */
cv::Mat make_ausgabe_screen (cv::Mat src, cv::Mat seg_screen)
{
    cv::Mat src_2;
    if (seg_screen.type() == CV_8UC1)
        cv::cvtColor (seg_screen, src_2, cv::COLOR_GRAY2BGR);       // Farbbild erzeugen => src2
    else 
        seg_screen.copyTo(src_2);                                   // seg_screen copy to => src2


    cv::Mat foo = cv::Mat (src.rows, src.cols + src_2.cols+10, CV_8UC3);    // Gesamtbild = src + seg_screen
    foo = cv::Scalar(255, 255, 255);                                        // Gesamtbild = white
    cv::Mat roi(foo, cv::Rect(0, 0, src.cols, src.rows));                   // ROI for src

    // Sensitiven Bildausschnitt zeichnen
    if ((geo.left != 0) || (geo.top != 0) || (geo.right != src.cols-1) || (geo.bottom != src.rows-1))
        cv::rectangle (src, 
                    cv::Point (geo.left, geo.top), cv::Point (geo.right, geo.bottom),
                    cv::Scalar(0, 0, 255), 
                    1);
                
    src.copyTo (roi);   // src ind Videobild eintragen
    cv::Mat roi2(foo, cv::Rect(src.cols+5, 0, src_2.cols, src_2.rows));     // ROI for seg_screen (src_2)
    src_2.copyTo (roi2);    // seg_screen in Videobild eintragen

    return foo;
}

/*! -------------------------------------------------------
 * 
 */
void write_diff_non_zero_to_diff ()
{
    char buf[256];

    sprintf (buf, "%i", properties.diff_non_zero);
    cv::putText(diff,                    // target image
                buf,                            // text
                cv::Point(10, 20),              // top-left position
                cv::FONT_HERSHEY_PLAIN,         // FONT_HERSHEY_PLAIN, FONT_HERSHEY_DUPLEX
                1.0,                            // fontScale
                255,                            // font color
                2);                             // thickness
}

/*! -------------------------------------------------
 * @brief   Funktion erzeugt aus <src> ein Mosaik
 */
void make_seg (cv::Mat src)
{
    int w = src.cols / HORZ_TEILER;
    int h = src.rows / VERT_TEILER;

    /* code berechnet den Rest der Fensterteilung. Wird z.Z. noch nicht benötigt. 
    int wr = src.cols % HORZ_TEILER;
    int hr = src.rows % VERT_TEILER;
    */

    // ------------ Mosaik einrichten -------------------   
    for (int y=0; y<VERT_TEILER; y++) {
        for (int x=0; x<HORZ_TEILER; x++) {
            seg[first_in][x][y] = src(cv::Rect(x*w, y*h, w, h));    // ROI einrichten
        }
    }

    // ------------- Differenz NonZero für jedes Mosaik-Bild ermitteln -----------
    seg_NonZero = 0;    // cv::Mat auf 0 setzen !!!
    for (int y=0; y<VERT_TEILER; y++) {
        for (int x=0; x<HORZ_TEILER; x++) {
            if (!seg[first_in][x][y].empty() && !seg[last_in][x][y].empty()) {
                cv::absdiff (seg[first_in][x][y], seg[last_in][x][y], seg_diff[x][y]);  // Bilder subtrahieren
                cv::threshold (seg_diff[x][y], seg_diff[x][y], properties.threshold, 255, THRESH_TOZERO);  // Pixel < properties.threshold = 0
                // -------- Nachschauen, ob Anzahl Farb-Pixel grösser ist als properties.NonZero_seg ---------
                if (cv::countNonZero(seg_diff[x][y]) > properties.NonZero_seg)  // --pixdiff für Mosaik, default 25
                    seg_NonZero.at<uchar>(y, x) = 128;
            }
        }
    }

    #define RESIZE_FAKTOR 40.0f
    // ------------------------- Contours ---------------------------------------------
    cv::resize (seg_NonZero, contours_pic, cv::Size(0, 0), RESIZE_FAKTOR, RESIZE_FAKTOR);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours (contours_pic, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE );
    std::vector<cv::Point> center;
    for (size_t i=0; i<contours.size(); i++) {
        cv::Moments m = cv::moments(contours[i]);
        center.push_back (cv::Point(m.m10 / m.m00, m.m01 / m.m00));
    }

    int cx = 0;
    for (size_t i=0; i<center.size(); i++) {
        cv::circle (contours_pic, center[i], 5, 255, 1);    
        cx += center[i].x;
    }
    if (cx != 0) 
        contour_x_center = cx / center.size();

    cv::line (contours_pic, cv::Point(contour_x_center, 0), cv::Point (contour_x_center, contours_pic.rows-1), 255, 1);

    char buf[256];
    sprintf (buf, "%ld  %2.1f%c", static_cast<long int>(contours.size()), (float)contour_x_center / (float)contours_pic.cols * 100.f, '%');
    cv::putText(contours_pic,                   // target image
                buf,                            // text
                cv::Point(10, 20),              // top-left position
                cv::FONT_HERSHEY_PLAIN,         // FONT_HERSHEY_PLAIN, FONT_HERSHEY_DUPLEX
                1.0,                            // fontScale
                255,                            // font color
                2);                             // thickness

#ifdef SHOW_MOSAIK
    // --------------- Mosaik - Bild erzeugen ---------------------
    show_seg = cv::Mat(h*VERT_TEILER + 5*VERT_TEILER+5,     // rows
                       w*HORZ_TEILER + 5*HORZ_TEILER+5,     // cols
                       CV_8UC3 );
    show_seg = cv::Scalar (255, 255, 255);
    for (int y=0; y<VERT_TEILER; y++) {
        for (int x=0; x<HORZ_TEILER; x++) {
            if (!seg_diff[x][y].empty()) {
                cv::Mat foo(show_seg, cv::Rect(x*w+5*x+5, y*h+5*y+5, w, h));
                cv::Mat dummy;
                seg[first_in][x][y].copyTo ( dummy );
                cv::cvtColor (dummy, dummy, cv::COLOR_GRAY2BGR);
                dummy.copyTo ( foo );
                if (seg_NonZero.at<uchar>(y, x))
                    cv::rectangle (show_seg, cv::Rect(x*w+5*x+5, y*h+5*y+5, w, h), cv::Scalar(0, 0, 255), 3);
            }
        }
    }
#endif
}

/*! -------------------------------------------------
 * @brief 
 */
void schwelle (cv::Mat &s, uint8_t schwellwert)
{
    #ifdef USE_CVD
        int foo = get_numval<int>(schwellwert, "schwaelle", 0, 255);
    #else 
        int foo = schwellwert;
    #endif

    for (int y=0; y<s.rows; y++) {
        uchar *p = s.ptr< uchar >(y);
        for (int x=0; x<s.cols; x++) {
            if (p[x] < foo) {
                p[x] = 0;
            }
        }
    }
}

/*! -------------------------------------------------
 * @brief   Bildeinzug und Bewegungserkennung.
 *
 * Die Bewegungserkennung arbeitet mit <absdiff()> \n
 * Sobald eine Bewegung erkannt wird, wechselt @ref <properties.falle_aktiv> auf true. \n
 * <properties.falle_aktiv> wird in @ref control() ausgewertet.
 */
void get_frame()
{
    first_in = (first_in < MAX_IN-1) ? first_in+1 : 0;  // Ringzähler weiterschieben
    last_in = (last_in < MAX_IN-1) ? last_in+1 : 0;
    properties.falle_aktiv = false;
    properties.frame_delay = MAX_DELAY;

    cap >> src_image;                               // Bildeinzug
    src[first_in] = src_image(cv::Rect(geo.left, geo.top, geo.right-geo.left+1, geo.bottom-geo.top+1)); // ROI
    now[first_in] = time(NULL);                         // Aufnahme-Zeitpunkt festhalten.

    cv::Mat gray;
    CVD::cvtColor (src[first_in], gray, cv::COLOR_BGR2GRAY);    // Graustufenbild

    // ----------------- stretch gray image -------------------
    Histogram1D h;

#ifdef SHOW_HISTOGRAM
    cv::Mat hist = h.getHistogram( gray );
    if (!properties.no_output)
        cv::imshow ("Hist", h.getImageOfHistogram(hist, 1.0f));     // Ausgabe original Histogram
#endif
    gray = h.stretch( gray, 0.0050f );                              // Histogram wird gestretcht.

#ifdef SHOW_HISTOGRAM
    hist = h.getHistogram( gray );
    if (!properties.no_output)
        cv::imshow ("Hist gestretcht", h.getImageOfHistogram(hist, 1.0f));   // Ausgabe gestretchtes Histogram
#endif
    // --------- End of stretch gray image --------------------

#ifdef USE_HARRIS_DETECTOR
    // ---------------------- Harris Corner -----------------------
    gray.copyTo (harrisCorners);
    harris.detect(gray);                                // Compute Harris corners
    std::vector<cv::Point> pts;
    harris.getCorners(pts, 0.02);                       // ermittle Koordinaten
    harris.drawOnImage(harrisCorners, pts, 255, 10, 2);  // Draw Harris corners
    cv::resize (harrisCorners, harrisCorners, cv::Size(), 0.5, 0.5);    // Bild verkleinern

    char buf[256];
    sprintf (buf, "%ld", static_cast<long int>(pts.size()));
    cv::putText(harrisCorners,                  // target image
                buf,                            // text
                cv::Point(10, 20),              // top-left position
                cv::FONT_HERSHEY_PLAIN,         // FONT_HERSHEY_PLAIN, FONT_HERSHEY_DUPLEX
                1.0,                            // fontScale
                255,                            // font color
                2);                             // thickness
#endif

#ifdef USE_FEATURE_DETECTOR
    gray.copyTo (harrisCorners);        // aktuelles Bild kopieren
    // -------------------- detector -----------------------
    pyrKeypoints.clear();
    detector->detect(gray, pyrKeypoints);

    std::vector<cv::Point> points;
    std::vector<cv::KeyPoint>::iterator it;
    for( it= pyrKeypoints.begin(); it!= pyrKeypoints.end(); it++)
        points.push_back(it->pt);

    harris.drawOnImage(harrisCorners, points, 255, 7, 1);  // Draw Harris corners
    // std::cout << "Interest points: Mat gray=" << pyrKeypoints.size() << std::endl;
#endif

    // ---------------------- smooth ------------------------
    cv::boxFilter (gray, gray, -1, cv::Size(6, 6));            // glätten: blur, gaussianBlur, filter2D, medianBlur, bilateralFilter, ...

    /* // ------------------- Versuch -----------------
    cv::Mat dummy;
    CVD::Laplacian (gray, dummy, CV_64F, 1, 1, 0);
    CVD::Sobel (dummy, dummy, CV_64F, 1, 1, 5);
    CVD::convertScaleAbs( dummy, gray );           // converting back to CV_8U
    */

    cv::dilate(gray, gray, Mat(), Point(-1, -1), 6, 1, 1);
    cv::erode(gray, gray, Mat(), Point(-1, -1), 6, 1, 1);
    cv::pyrDown (gray, gray, cv::Size(0, 0));
    make_seg (gray);
    cv::pyrDown (gray, gray, cv::Size(0, 0));      // gray enthält das runter gebrochene Bild !!!

    gray.copyTo (in[first_in]);                     // dieser Schritt könnte im letzten pyrDown() eingebunden werden.

    if (!in[last_in].empty()) {
        /*
        cv::Mat akt = in[first_in] (cv::Rect(geo.left, geo.top, geo.right-geo.left+1, geo.bottom-geo.top+1));
        cv::Mat vor_akt = in[last_in] (cv::Rect(geo.left, geo.top, geo.right-geo.left+1, geo.bottom-geo.top+1));
        cv::absdiff (akt, vor_akt, diff);      // Differenzbild berechnen => diff
        */
        cv::absdiff (in[first_in], in[last_in], diff);      // Differenzbild berechnen => diff

        // schwelle (diff, properties.threshold);   // Ersetzt durch threshold. Siehe nächste Zeile
        cv::threshold (diff, diff, properties.threshold, 255, THRESH_TOZERO);

        anz_zero[first_in] = cv::countNonZero(diff);                        // Anzahl nonZero-Pixel in <diff> ermitteln.
        properties.diff_non_zero = anz_zero[last_in] - anz_zero[first_in];  // Differenz zum Vorgängerbild berechnen.
        if (abs(properties.diff_non_zero) >= properties.video_start_diff) { // Hat es eine groessere Differenz ergeben ?
            properties.falle_aktiv = true;      // Bewegung erkannt. Video kann gestartet werden.
            properties.frame_delay = MAX_DELAY;
        }
    }

    usleep (properties.frame_delay);     
}

/*! -------------------------------------------------
 * @brief state-machine kontrolliert den Videostream.
 *
 * Im wesentlichen werden Frameänderung und Framespeicherung abgearbeitet. \n
 * Im Idle-Mode werden Frameänderungen erkannt aber nicht gespeichert. \n
 * Gesteuert wird die state-machine durch die Variable @ref state.
 */
static void control()
{
    char fname[512];
    static char pic_name[512];
    static int frame_counter = 0;       // Dient zum Zählen der abgespeicherten frames.
    static int nachlauf_counter = 0;

    get_frame();        // Bildeinzug und Bewegungserkennung. Wenn eine Bewegung erkannt wurde, wird <falle_aktiv> TRUE
    switch (state) {
        case 0: // --------------- idle - state ------------------
            if (!properties.run) {      // ---- Überwachung ist inaktiv ----
                if (properties.falle_aktiv) {
                    static int n = 0;
                    if (!properties.no_output) cout << " Bewegung erkannt(" << n << "): " << properties.diff_non_zero << endl;     // Anzeigen, das die Falle eine Bewegung erkannt hat !!!
                    ++n;
                    properties.falle_aktiv = false;
                    properties.diff_non_zero = 0;
                }
                break;
            } else {                    // ---- Überwachung ist aktiv ----
                if (properties.falle_aktiv) {       // Falle ist aktiviert. Siehe <get_frame()>. 
                    if (!properties.no_output) cout << "now: " << properties.diff_non_zero << endl;

                    // int schwelle = 8000; 
                    const int schwelle = (int)((float)(back.cols * back.rows) * 0.41666666666);     // 41,6% von back
                    int delta = schwelle + 1;

                    if (!back.empty()) {    // es ist ein Hintergrundbild vorhanden !!!
                        cv::Mat d;
                        cv::absdiff (back, in[first_in], d);
                        delta = cv::countNonZero ( d );         // Anzahl der NICHT schwarzen Pixel ermitteln.
                        if (!properties.no_output) cout << "diff back-in " << delta << endl;
                    }

                    if (delta > schwelle) { 
                        nachlauf_counter = 0;
                        if (!back.empty())
                            back.release();     // Hintergrundbild löschen

                        timefunc::stop_timer ("CONTROL");
                        state = 100;        // Aufnahme starten
                    } else {
                        properties.falle_aktiv = false;
                        properties.diff_non_zero = 0;
                    }
                } else { 
                    if (abs(properties.diff_non_zero) > 0) {         // noch keine aktive Falle aber es sind Differenz-Pixel vorhanden
                        if (!properties.no_output) cout << properties.diff_non_zero << endl;
                    } else {
                        if (nachlauf_counter > 8)           // nach 8 Bilder mit <diff_non_zero == 0> wird der background festgehalten !!
                            in[first_in].copyTo(back);      // save in[fist_in] at back-screen
                        else 
                            ++nachlauf_counter;
                    } 
                }
            }
            break;
        case 100: {
                // --------------- Dateiname berechnen ------------------
                if (!properties.only_picture)                               // video Mode
                    sprintf (fname, "%s/out%i.avi", folder.c_str(), vid_counter);   // Dateiname ermitteln
                else {                                              // only picture Mode
                    sprintf (fname, "%s/out_picture.avi", folder.c_str());  // Frame Dateiname ermitteln
                    sprintf (pic_name, "%s/out%i.jpg", folder.c_str(), vid_counter);    // Picture Dateiname
                }

                ++vid_counter;

                // ---------------- Video-Datei öffnen ----------------
                // cv::Mat foo = make_ausgabe_screen(src[last_in], show_seg);  // Bildgroesse ermitteln
                cv::Mat foo = make_ausgabe_screen(src_image, contours_pic);  // Bildgroesse ermitteln

                bool ret = sv.open ( fname, foo.cols, foo.rows );   // Datei mit entsprechender Bildgroesse oeffnen !
                if (!ret)
                    cout << "cant open " << fname << endl;

                frame_counter = 0;
                state = 110;
                timefunc::start_timer ("CONTROL");
            }
            break;
        case 110: {
                // ---------------- Bilder speichern -------------------
                char buf[256];
                sprintf (buf, "%i pix", abs(properties.diff_non_zero));

                sv.write( make_ausgabe_screen(src_image, contours_pic),  &now[last_in], buf );     // Bild im Video ablegen !!!
                cout << "." << flush;       // Fortschrittsanzeige
                ++frame_counter;

                if (properties.only_picture) {      // Bilder speichern
                    if (frame_counter == 3) {
                        // -------- Support for writing JPG ----------
                        vector<int> compression_params;
                        compression_params.push_back( cv::IMWRITE_JPEG_QUALITY );
                        compression_params.push_back( 100 );
                        // cv::Mat out (src[last_in]);
                        cv::Mat out (src_image);
                        if (sv.get_gray_flag() == true)
                            cv::cvtColor (out, out, cv::COLOR_BGR2GRAY);               // Graustufenbild

                        sv.write_date_to_pic (out, &now[last_in], buf);     // Zeit ins Bild schreiben
                        cv::imwrite (pic_name, out, compression_params);    // save image
                    }
                }

                if (properties.falle_aktiv) {
                    if (timefunc::get_timer("CONTROL") >= properties.max_time) {    // max.Anzahl Bilder erreicht. 
                                                                                    // Goto close Viedeo. 
                                                                                    // Es findet kein Nachlauf statt !!!
                        state = 130;                // Goto close Video
                    }
                } else if (timefunc::get_timer("CONTROL") > properties.min_time - (170 * properties.trail)) {    // Es ist keine Bewegung erkannt worden und 
                                                    // die Anzahl der Bilder ist > 10. 
                                                    // 10 Bilder benötigen ca. 1700 ms.
                    nachlauf_counter = 0;
                    state = 120;                    // Goto Video Nachlauf
                }
            }
            break;
        case 120: {  // ------------ video Nachlauf ca. 1200 ms ----------------------
                char buf[256];
                sprintf (buf, "%i pix", abs(properties.diff_non_zero));

                sv.write ( make_ausgabe_screen(src_image, contours_pic),  &now[last_in], buf );     // Bild im Video ablegen !!!
                cout << "." << flush;       // Fortschrittsanzeige
                ++frame_counter;
                ++nachlauf_counter;

                if ((nachlauf_counter > properties.trail) || (timefunc::get_timer("CONTROL") > properties.max_time)) 
                    state = 130;        // close video
            }
            break;
        case 130:   // ------------------ close video --------------------------
            sv.close();
            cout << endl;
            // cout << "Aufnahmedauer = " << timefunc::stop_timer("CONTROL") << " ms" << endl;
            frame_counter = 0;
            state = 0;
            break;
    }
}

/*! ------------------------------------------------------------
 * @brief VideoCaptureProperties werden gelesen\n
 *        Vor dem lesen werden noch Bildbreite und Bildhöhe eingestellt.\n 
 * @see {@ref show_cam_para()}
 */
static void get_cam_para ()
{
    cap.set(cv::CAP_PROP_FRAME_WIDTH, camwidth);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, camheight);

    // ------------------ Camera Parameter ----------------------------
    saturation = cap.get (cv::CAP_PROP_SATURATION);      // 1
    brightness = cap.get (cv::CAP_PROP_BRIGHTNESS);      // 0
    contrast = cap.get (cv::CAP_PROP_CONTRAST);          // 1
    exposure = cap.get (cv::CAP_PROP_EXPOSURE);          // 157
    fwidth = cap.get (cv::CAP_PROP_FRAME_WIDTH);
    fheight = cap.get (cv::CAP_PROP_FRAME_HEIGHT);
}

/*! ------------------------------------------------------------
 * @brief Ausgabe der VideoCaptureProperties
 * @see {@ref get_cam_para()}
 */
static void show_cam_para ()
{
    cout << "--------- cam para ----------\n";
    cout << "saturation: " << saturation << endl;
    cout << "brightness: " << brightness << endl;
    cout << "contrast: " << contrast << endl;
    cout << "exposure: " << exposure << endl;
    cout << "fwidth: " << fwidth << endl;
    cout << "fheight: " << fheight << endl;
}

/*! ------------------------------------------------------------
 * 
 */
int main (int argc, char ** argv)
{
    show_cplus_version ();
    control_opt(argc, argv);    
    check_plausibiliti_of_opt ();

    init_keyboard ();           // wird für kbhit() benötigt !
    get_homedir();              // Home Verzeichnis ermitteln.
    init_folder();              // Pfad für Video-Speicherung einrichten.
    init_vid_counter();         // Video-Nr ermitteln !

    if (!properties.no_output) {
        cout << "version: " << VERSION << endl;
        cout << "Camera Index: " << properties.cam_index << endl;
    }

    // -------------- create win-name ------------------------
    char src_win_name[256];
    sprintf (src_win_name, "%s %s", "src_image", VERSION);  // Text für Window Titelleiste

    if (!properties.no_output) {
        cv::namedWindow("diff_image");
        cv::namedWindow(src_win_name);      // Fenster für src[first_in]
        // cv::namedWindow("Harris");
        // cv::namedWindow("back_image");
    }

    // cap = new cv::VideoCapture ( properties.cam_index );
    cap.open ( properties.cam_index, cv::CAP_V4L2 );

    if(!cap.isOpened()) {           // check if we succeeded
        cout << "NO CAMERA\n";
        return -1;
    }

    get_cam_para ();
    reset_geo ();

    show_geo ();
    show_short_keys ();
    show_cam_para ();

    // ------------------- Bildeinzug initialisieren --------------------
    while (diff.empty())    
        get_frame();

    // -------------- Verweilzeit -------------------------------------
    for (int i=0; i<10; i++) {
        get_frame();
        cout << "+" << flush;
    }
    cout << endl;

    int key = -1;
    int ende = 0;
    while (!ende) {
        control();      // Betriebszustände überwachen.

        // ----------------- Bildausgabe ------------------------
        if (!properties.no_output) {
            if (!diff.empty()) {
                write_diff_non_zero_to_diff ();
                cv::imshow ("diff_image", diff);
            }

            if (!src_image.empty()) {
                cv::rectangle (src_image, // src[first_in], 
                               cv::Point (geo.left, geo.top), cv::Point (geo.right, geo.bottom),
                               cv::Scalar(0, 0, 255), 
                               3);
                cv::imshow (src_win_name, src_image);
            }

            #ifdef SHOW_MOSAIK
            if (!show_seg.empty())
                cv::imshow("Mosaik", show_seg);
            #endif

            #ifdef USE_HARRIS_DETECTOR
            if (!harrisCorners.empty())
                cv::imshow("Harris", harrisCorners);
            #endif

            if (!contours_pic.empty())
                cv::imshow("contours", contours_pic);

            // ------------------- back image ----------------------
            if (!back.empty())
                cv::imshow ("back_image", back);
            // ---- destroyWindow funktioniert nicht auf dem raspi -----
            /* else 
                cv::destroyWindow("back_image"); */
        }

        // --------------- Tastatur abfragen ------------------------
        if ((key = cv::waitKey(10)) == -1) {    // key im opencv-window abfragen.
            if (kbhit()) {                      // key im terminal abfragen.
                key = getch();
            }
        } 

        // --------------- Tastendruck auswerten --------------------
        if ((key == 27) || (key == 'q'))
            ende = 1;
        if (key == 'h')
            show_short_keys();
        if (key == 'm') {
            cout << ((properties.run) ? "run inaktiv\n" : "run aktiv\n");
            properties.run = !properties.run;
        }
        if (key == 'i') {
            show_properties ();       // properties anzeigen
            show_geo ();        // struct _geo_ anzeigen.
            show_cam_para ();   // Camera Parameter
        }
        if (key == 'f')
            sv.show_fileliste();
    }

    close_keyboard ();
    return 0;
}

//! @} main