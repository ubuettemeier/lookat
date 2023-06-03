/*! ------------------------------------------
 * @defgroup log_data Logdata: Modul zum verwalten von Log-Einträgen.
 * @{
 */

/*! -----------------------------------------------------
 * @brief   Program erstellt und verwalten eine Log-Datei.
 * @file    error_class.hpp
 * @author  Ulrich Büttemeier, Stemwede, DE
 * @version v0.2.2
 * @date    2022-05-14
 * @todo    
 *          - Eventuell <add_log> in einem Thread ausführen.\n
 *            Thread hat den Nachteil, das evnetuell log's nicht abgespeichert werden.
 * 
 * @copyright Copyright (c) 2022
 */

#define ERROR_CLASS_VERSION "v0.2.2"

#ifndef ERROR_CLASS_HPP
#define ERROR_CLASS_HPP

#include <iostream>
#include <string>
#include <cstring>
#include <algorithm>
#include <stdio.h>
#include <fstream>
#include <mutex>
#include <vector>
#include <thread>
#include <chrono>

#ifdef _WIN32
    #include <sys\time.h>
#elif  __linux__
    #include <sys/time.h>
#else
    #error Platform not suppoerted
#endif

using namespace std;

namespace { 
    mutex mtx;      // see: <add_log>
}

#define BUILDIN  int line_nr = __builtin_LINE(), \
                 const char *src_file = __builtin_FILE() 

struct _error_data_ {
    struct timeval tv;
    string text;
    uint16_t error_nr;
    uint16_t error_group;
    int line_nr;
    string scr_file;
};

struct _flags_error_class_ {
    bool show_on_screen;        // default=0  log-Text wird auch auf der Console angezeigt.
    bool save_in_file;          // default=1  log-Text wird gespeichert.
    bool with_date_and_time;    // default=1  Datum und Zeit wird im log-Text angezeigt.
    bool with_error_no;         // default=1  Error-Nr wird eingebunden.
    bool with_group;            // default=1  Guppen-Nr wird eingebunden.
    bool with_src_file;         // deafult=1  Zeigt an, in welchen file der log generiert wurde.
    bool with_line_nr;          // default=1  Gibt die Zeilen-Nr aus
    bool with_zeilen_counter;   // default=0  Trägt die Zeilen-Nr mit ein.
    uint8_t error_nr_format;    // Ausgabeformat für error_nr.
                                // default=<hex_ausgabe>, see: <enum error_no_ausgabe_formate>
};

const string group_text[] {"warning", "info", "error", "fatal_error"};

/** -----------------------------------------------------------------------------
 * @brief
 */
class error_log {
public:
    enum error_no_ausgabe_formate {
        hex_ausgabe = 1,
        dez_ausgabe = 2,
    };

    enum classification {   // group's
        warning = 0,        // schlechtes Passwort
        info = 1,
        error = 2,          // inkorrekte Eingabe, Passwort enthält kein Sonderzeichen.
        fatal_error = 3,    // Stop error, z.B. divide by zero
    };

    error_log() = delete;   // no default Konstruktor
    ~error_log() = delete;

    error_log(error_log&) = delete;         
    void operator=(error_log&) = delete;

    static int add_log ( const char *text, 
                         uint16_t error_nr=0x0000, 
                         uint16_t error_group=0x0000,
                         BUILDIN );

    static int remove_log_file();
    static char *get_version();

    static void buffer_thread_quit();           // beendet <buffer_thread()>
    static void buffer_thread();                // thread: puffert die log's
    static void set_buffer_thread (bool val);   // schaltet den puffer-thread EIN/AUS

public:
    // Properties
    static int max_anzahl_logs;         // -1: keine Begrenzung
    static int anzahl_logs;             // Enthält die Anzahl der Zeilen in der Log-Datei !!!
                                        // Wird mit -1 initialisiert.
    static int anzahl_delete_logs;      // default=1: Anzahl der zu löschenden lines, 
                                        // wenn die Datei <max_anzahl_logs> überschreitet.
    static string log_file_name;        // Default: "log.dat"
    // flags
    static struct _flags_error_class_ flag;    

private:    
    static int get_anzahl_zeilen (const char *fname);                   // Funktion ermittelt die Anzahl der Zeilen der Detei: log_file_name    
    static int delete_log_line (const char *file_name, int n);          // delete first n line's.
    static int save_log_line (const char *text, const char *fname);
    static int write_log (struct _error_data_ ed);

    static thread *gothread;
    static bool use_buffer_thread;                      // flag
    static uint8_t buffer_thread_ende;                  // spiegelt den thread-status wider
    static uint8_t buffer_lock;                         // atomic
    static vector <struct _error_data_> error_data;     // log puffer
};

// Initialisierungs
string error_log::log_file_name = "log.dat";
int error_log::max_anzahl_logs = -1;            // -1: keine Begrenzung. >0: max. n log-lines
int error_log::anzahl_logs = -1;                // Enthält die Anzahl der Zeilen in der Log-Datei !!!
int error_log::anzahl_delete_logs = 1;          // Anzahl der zu löschenden lines, 
                                                // wenn die Datei <max_anzahl_logs> überschreitet.
thread *error_log::gothread = nullptr;
bool error_log::use_buffer_thread = 0;
uint8_t error_log::buffer_thread_ende = 0;
uint8_t error_log::buffer_lock = 0;
vector <struct _error_data_> error_log::error_data;

// ----------- flags ---------------
struct _flags_error_class_ error_log::flag = {.show_on_screen = 0,
                                  .save_in_file = 1,
                                  .with_date_and_time = 1,
                                  .with_error_no = 1,
                                  .with_group = 1,
                                  .with_src_file = 1,
                                  .with_line_nr = 1,
                                  .with_zeilen_counter = 0,
                                  .error_nr_format = hex_ausgabe};

/** -----------------------------------------------------------------------------
 * @brief 
 */
int error_log::save_log_line (const char *buf, const char *fname)
{
    int ret = EXIT_SUCCESS;
    ofstream outfile;

    // ----------------- C++ Methode ------------------
    outfile.open (fname, ios::app);     // ios::app Daten anhaengen
    if (outfile.is_open()) {
        outfile << buf << endl;         // ERROR-String eintragen
        outfile.flush();
        outfile.close();
    
        // Nachschauen, ob max.Anzahl logs erreicht sind !
        if (max_anzahl_logs > 0) {
            if (anzahl_logs >= 0)
                ++anzahl_logs;
            else
                anzahl_logs = get_anzahl_zeilen(fname);

            if (anzahl_logs > max_anzahl_logs) {    // log-Datei muss verkleinert werden !
                int delta = ((anzahl_logs - max_anzahl_logs) > anzahl_delete_logs) ? 
                            anzahl_logs - max_anzahl_logs : anzahl_delete_logs;

                delete_log_line ( fname, delta );
            }
        }
    } else {
        ret = EXIT_FAILURE;
    }

    return ret;
}

/** -----------------------------------------------------------------------------
 * @brief 
 */
void error_log::set_buffer_thread (bool val)
{
    use_buffer_thread = val;

    if (use_buffer_thread) {
        if (gothread == nullptr) {
            buffer_thread_ende = 0;
            gothread = new thread( &error_log::buffer_thread );
            gothread->detach();
        }
    } else {
        if (gothread != nullptr)
            buffer_thread_quit();
    }
}

/** -----------------------------------------------------------------------------
 * @brief   
 */
void error_log::buffer_thread_quit()
{
    buffer_thread_ende = 1;

    int n = 0;
    while ((buffer_thread_ende == 1) && (n < 10)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ++n;
    }

    gothread = nullptr;

    // cout << "check_buffer_quit\n";
}

/** -----------------------------------------------------------------------------
 * @brief 
 */
void error_log::buffer_thread()
{
    // cout << "-- error_log thread gestartet\n";
    atexit (error_log::buffer_thread_quit);

    while (!buffer_thread_ende) {
        if (!buffer_lock) {
            if (error_data.size()) {
                // cout << "kill data\n";
                write_log ( error_data[0] );
                error_data.erase ( error_data.begin() );
            }
        }

        if (!buffer_thread_ende)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    buffer_thread_ende = 2;
}

/** -----------------------------------------------------------------------------
 * @brief 
 */
int error_log::write_log (struct _error_data_ ed) 
{
    int ret = EXIT_SUCCESS;

    char tmbuf[64], usecbuf[64], errornobuf[64], groupbuf[64];
    char scr_file_buf[512], line_nr_buf[32];
    char zeile_nr_buf[32];
    char buf[4096];

    // Ist-Zeit ermitteln
    if (flag.with_date_and_time) {
        struct tm *nowtm;
        time_t nowtime = ed.tv.tv_sec;
        nowtm = localtime( &nowtime );

        strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm );
        snprintf (usecbuf, sizeof(usecbuf), ".%03ld: ", ed.tv.tv_usec/1000 );
    } else {
        tmbuf[0] = '\0';
        usecbuf[0] = '\0';
    }

    // Error-Nr eintragen
    if (flag.with_error_no) {
        if (flag.error_nr_format == hex_ausgabe) 
            snprintf (errornobuf, sizeof(errornobuf), "#%04X: ", ed.error_nr);
        if (flag.error_nr_format == dez_ausgabe) 
            snprintf (errornobuf, sizeof(errornobuf), "#% 6d: ", ed.error_nr);
    } else
        errornobuf[0] = '\0';

    // group eintragen
    if (flag.with_group) {
        snprintf (groupbuf, sizeof(groupbuf), "%s: ", group_text[ed.error_group].c_str());
    } else 
        groupbuf[0] = '\0' ;

    // line_nr eintragen
    if (flag.with_line_nr)
        snprintf (line_nr_buf, sizeof(line_nr_buf), "%i: ", ed.line_nr);
    else 
        line_nr_buf[0] = '\0';

    // src file eintragen
    if (flag.with_src_file)
        snprintf (scr_file_buf, sizeof(scr_file_buf), "%s: ", ed.scr_file.c_str() );
    else 
        scr_file_buf[0] = '\0';

    if (flag.with_zeilen_counter) {
        int foo = get_anzahl_zeilen(log_file_name.c_str());
        if (foo == -1) foo = 0;
        snprintf (zeile_nr_buf, sizeof(zeile_nr_buf), "% 4i> ", foo+1);
    } else
        zeile_nr_buf[0] = '\0';

    // create log-string. Max. 4096 Zeichen
    snprintf(buf, sizeof buf, "%s%s%s%s%s%s%s%s", zeile_nr_buf, 
                                                    scr_file_buf, 
                                                    line_nr_buf, 
                                                    tmbuf, usecbuf, 
                                                    errornobuf, 
                                                    groupbuf, 
                                                    ed.text.c_str() ); 

    // Ausgabe Console
    if (flag.show_on_screen)
        cout << buf;

    // Ausgabe Logfile 
    if (flag.save_in_file) {
        if (save_log_line (buf, log_file_name.c_str()) == EXIT_FAILURE) {
            cout << "kann error nicht in <" << log_file_name << "> speichern\n";
            if (save_log_line (buf, "log.dat") == EXIT_FAILURE) {
                cout << "kann error nicht speichern\n";
            } else {
                log_file_name = "log.dat";
                cout << "neue Log-Datei: " << log_file_name << endl;
            }
        }
    }

    return ret;
}

/*! -----------------------------------------------------------------------------
 * @brief   Es wird ein neuer Text im log_file eingetragen.
 * @param   text        ERROR-String
 * @param   src_file    Quelle: Source-file
 */
int error_log::add_log ( const char *text, uint16_t error_nr, uint16_t error_group, 
                         int line_nr, 
                         const char *src_file )
{
    lock_guard<mutex> lock(mtx);    // race condition verhindern
    int ret = EXIT_SUCCESS;

    struct _error_data_ ed;

    gettimeofday(&ed.tv, NULL);
    ed.text = text;
    ed.error_nr = error_nr;
    ed.error_group = error_group;
    ed.line_nr = line_nr;
    ed.scr_file = src_file;

    if (use_buffer_thread) {
        if (gothread == nullptr) {
            buffer_thread_ende = 0;
            gothread = new thread( &error_log::buffer_thread );
            gothread->detach();
        }

        buffer_lock = 1;
        error_data.push_back ( ed );
        buffer_lock = 0;
    } else {
        write_log (ed);
    }
    return ret;
}

/** -----------------------------------------------------------------------------
 * @brief   Delete first n line's from given file
 * @param   n: Anzahl der zu löschenden Zeilen.
 */
int error_log::delete_log_line (const char *file_name, int n)
{
    // open file in read mode
    ifstream is(file_name, ofstream::in);
    if (!is.is_open())
        return EXIT_FAILURE;
  
    // open file in write mode
    ofstream ofs("temp.txt", ofstream::out);
    if (!ofs.is_open()) {
        is.close();
        return EXIT_SUCCESS;
    }
  
    // copy in-file to out-file without n first lines
    int line_no = 0;
    string strline;
    while (std::getline(is, strline)) {
        ++line_no;
        if (line_no > n) 
            ofs << strline << endl;     // copy input-string to output-string
    }
    anzahl_logs = ((anzahl_logs - n) >= 0) ? anzahl_logs-n : 0;

    // closing output file
    ofs.close();
    
    // closing input file
    is.close();

    // remove the original file
    std::remove(file_name);
  
    // rename the temp-file
    std::rename("temp.txt", file_name);

    return EXIT_SUCCESS;
}

/** -----------------------------------------------------------------------------
 * @brief       Funktion ermittelt Anzahl der Zeilen in der Datei <fname>
 * @attention   Eventuell Funktion nach Programm-Start einmal aufrufen.
 *              Anschließend den Internen Counter weiterschieben.
 * @return      -1: kein Ergebniss, d.h. Deitei nicht vorhanden, ...
 *              >=0: Zeilen gelesen.
 */
int error_log::get_anzahl_zeilen (const char *fname)
{
    int ret = -1;

    // Anzahl Zeilenumbrüche zählen.
    std::ifstream inFile(fname);
    if (inFile.is_open()) {
        ret = std::count(std::istreambuf_iterator<char>(inFile), 
                         std::istreambuf_iterator<char>(), '\n');
        inFile.close();
    }
    
    return ret;
}

/** -----------------------------------------------------------------------------
 * @brief 
 */
int error_log::remove_log_file()
{
    return remove (log_file_name.c_str());
}

/** -----------------------------------------------------------------------------
 * @brief 
 */
char *error_log::get_version()
{
    static char buf[256] = ERROR_CLASS_VERSION;
    return buf;
}

//! @} log_data

#endif
