/*! ------------------------------------------
 * @defgroup Save_Vid Save_Vid: Klasse zum speichern von Video's
 * @{
 * 
 * @file    Save_Vid.hpp
 * @author  Ulrich Buettemeier
 * @date    2021-11-28
 * @brief   Class zum speichern von Video's.\n
 * Ein Video wird durch speichern der Einzelbilder erstellt.\n 
 * Eine Graustufen-Konvertierung ist möglich.
 * 
 * @copyright Copyright (c) 2021, 2022, 2023 Ulrich Buettemeier, Stemwede
 */

#ifndef SAVE_VID_HPP
#define SAVE_VID_HPP

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <vector>

#include "opencv2/opencv.hpp"

#define USE_CVD_
#ifdef USE_CVD
    #include "../../OpenCVD/include/opencvd.hpp"
#endif

using namespace std;
using namespace cv;

/*! -------------------------------
 * @brief class for save video-data.
 */
class save_video {
public:
    save_video (): frame_counter(0), make_gray(false) {}
    ~save_video ();
    int open(std::string fname, int w=640, int h=480);      // open the video
    void close ();
    void write(cv::Mat src, time_t *ext_now = NULL, char *str = NULL, bool draw_date = true);
    int get_frame_counter() {return frame_counter;}     // Get the frame counter object
    int set_gray (bool gray_vid);
    bool get_gray_flag ();
    void write_date_to_pic (cv::Mat &src, time_t *ext_now = NULL, char *str = NULL); 

    void set_maxvideo (int wert);
    void show_fileliste ();

private:
    cv::VideoWriter *vw = NULL;         //!< Pointer wird in @ref open() erzeugt. Freigegeben wird er in @ref close().
    int width;
    int height;
    int frame_counter = 0;
    int maxvideo = -1;                  //!< Maximale Anzahl an Videodateien. \n Wird die Anzahl überschritten, wird die erste Datei gelöscht. \n Bei maxvideo = -1 gibt es keine Begrenzung.
    std::string akt_fname;
    vector <std::string> file_liste;    //!< Liste enthält die Dateinamen.
    bool make_gray = false;             //!< bei true wird das Bild/Video als Graustufe gespeichert. @see @ref set_gray()
};

/*! ----------------------------------------------
 * @brief Destroy the save vid object
 */
save_video::~save_video () 
{
    if (vw != NULL)     // vw = pointer to VideoWriter
        vw->release();
    vw = NULL;
}

/*! ----------------------------------------------
 * @brief   open the video\n
 *          H.264  funktioniert auf dem Raspi nicht ! \n
 *          MJPG  macht keine gray videos. \n 
 *          Der Pointer @ref vw zeigt auf cv::VideoWriter().
 */
int save_video::open(std::string fname, int w, int h) 
{
    bool ret = true;
    if (vw != NULL)     // vw = pointer to VideoWriter
        close();

    width = w;
    height = h;
    frame_counter = 0;
    double fps = 5.0f;  // 10.0f    // Framerate of the created video stream. 
                                    // Das Video wird später mit dieser Geschwindigkeit abgespielt.
    
    // --- H.264 Funktioniert nicht auf raspi ---
    // vw = new cv::VideoWriter(fname, VideoWriter::fourcc('H','2','6','4'),       // used by vlc (color, grayscale). 
    vw = new cv::VideoWriter(fname, VideoWriter::fourcc('M','J','P','G'),   
                                fps, 
                                Size(width, height), 
                                !make_gray);
    if (vw == NULL) {
        ret = false;
        akt_fname.clear();
    } else 
        akt_fname = fname;

    return ret;
}

/*! ----------------------------------------------
 * @brief close the video\n
 * Pointer @ref vw wird freigegeben.
 */
void save_video::close()
{
    if (vw != NULL)     // vw = pointer to VideoWriter
        vw->release();

    vw = NULL;
    if (!akt_fname.empty())
        file_liste.push_back (akt_fname);

    if (maxvideo >= 0) {
        while (file_liste.size() > (size_t)maxvideo) {      // Max. Anzahl der Dateien überschritten. 
            std::remove (file_liste[0].c_str());            // Datei löschen
            file_liste.erase(file_liste.begin());           // Eintrag 0 aus Liste löschen
        }
    }
}

/*! ----------------------------------------
 * @brief Variable @ref maxvideo wird ein Wert zugewiesen.
 * @param wert Neuer Wert für @ref maxvideo.
 */
void save_video::set_maxvideo (int wert)
{
    maxvideo = wert;
}

/*! --------------------------------
 * @brief Die Fileliste wird im Terminal angezeigt.
 */
void save_video::show_fileliste ()
{
    cout << "------ Fileliste Max=" << maxvideo << " Ist=" << file_liste.size() << " --------\n";
    for (size_t i=0; i<file_liste.size(); i++) {
        cout << file_liste[i] << endl;
    }

    cout << endl;
}

/*! ----------------------------------------------
 * @brief write the frame to the video\n
 * Sollte das Flag {@ref make_gray} gesetzt sein, wird ein Graustufenvideo erstellt.
 * @param src Picture to save in Video
 * @param ext_now Zeitstempel
 * @param str optionaler Zusatztext
 * @param draw_date \n 
 *                  1: Datum im Bild eintragen\n
 *                  0: kein Datum eintragen.
 * 
 */
void save_video::write(cv::Mat src, time_t *ext_now, char *str, bool draw_date)
{
    if (vw == NULL)     // vw = pointer to VideoWriter
        return;

    cv::Mat out;
    cv::resize (src, out, Size(width, height), INTER_LINEAR);       // resize video

    if (make_gray) 
        cv::cvtColor (out, out, cv::COLOR_BGR2GRAY);               // Graustufenbild

    if (draw_date) {
        write_date_to_pic (out, ext_now, str);      // Zeit ins Bild schreiben
    }
    // ------------------- Bild speichern --------------------------------
    vw->write (out);       // write video
    ++frame_counter;
}

/*! ----------------------------------------------
 * @brief   Set the gray flag
 * @param gray_vid New state for @ref make_gray.
 * @return  Immer EXIT_SUCCESS
 */
int save_video::set_gray (bool gray_vid) 
{
    make_gray = gray_vid;
    return EXIT_SUCCESS;
}

/*! ----------------
 * @brief
 */
bool save_video::get_gray_flag ()
{
    return make_gray;
}

/*! ----------------------------------------------
 * @brief Zeitstempel und Zusatztext im Bild eintragen
 * @param src Picture 
 * @param ext_now Zeitstempel
 * @param str optionaler Zusatztext
 */
void save_video::write_date_to_pic (cv::Mat &src, time_t *ext_now, char *str)
{
    char buf[512];
    // ------------------- Zeit ermitteln ----------------------
    struct tm t;
    time_t now;

    if (ext_now == NULL)
        now = time(NULL);
    else 
        now = *ext_now;

    localtime_r (&now, &t);
    char str_buf[256];
    strcpy (str_buf, (str == NULL) ? "" : str);
    sprintf (buf, "%i / %02i.%02i.%i | %02i:%02i:%02i | %s", frame_counter, 
                                            t.tm_mday, t.tm_mon+1, t.tm_year+1900, 
                                            t.tm_hour, t.tm_min, t.tm_sec,
                                            str_buf);
    // ------------------ Zeitstempel im Bild eintragen ----------------------
    cv::putText(src,                    // target image
        buf,                            // text
        cv::Point(10, 20),              // top-left position
        cv::FONT_HERSHEY_PLAIN,         // FONT_HERSHEY_PLAIN, FONT_HERSHEY_DUPLEX
        1.0,                            // fontScale
        (make_gray) ? 255 : CV_RGB(118, 185, 0),   // font color
        2);                             // thickness
}

#endif

//! @} main