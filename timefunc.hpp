/*! ------------------------------------------
 * @defgroup timefunc Timefunc: Modul mit Zeitfunktionen.
 * @{
 */

/*! ------------------------------------------------------------
 * @brief   timefunc.hpp enthaelt Funktionen zum messen von Zeiten
 * @file    timefunc.hpp
 * @author  Ulrich Büttemeier, Stemwede, DE
 * @version 0.1.2
 * @date    2022-10-02
 * 
 * @copyright Copyright (c) 2022-2023, Ulrich Büttemeier, Stemwede, DE
 * 
 * @code
 * timefunc::start_timer ("my_timer");
 * .
 * .
 * int zeit = timerfunc::stop_timer ("my_timer");
 * @endcode
 */

#ifndef TIMEFUNC_HPP
#define TIMEFUNC_HPP

// #define AUSGABE_TIMER_TEXT

#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <sys/time.h>

#include "error_class.hpp"

using namespace std;

#define MAX_TIMER 10

struct _timeval_ {
    char timer_name[128];
    struct timeval start;
    uint64_t dauer;
};

/*! -----------------------------------------------------------------------------
 * @brief Classe stellt Funktionen für Zeitmessungen zur Verfuegung.
 */
class timefunc {
public:    
    timefunc() = delete;        //!< Konstruktor nicht erlaubt
    ~timefunc() = delete;

    timefunc(timefunc&) = delete;
    void operator=(timefunc&) = delete;

    static int start_timer (const char *timer_name);
    static int stop_timer (const char *timer_name);
    static int get_timer (const char *timer_name);

    static void list_timer();

    static int difference_micro (struct timeval *start, struct timeval *stop);
    static int difference_milli (struct timeval *start, struct timeval *stop);

private:
    static int grep_timer (const char *timer_name);
    static int get_time (const char *timer_name, uint8_t flag = 0);

    static vector <struct _timeval_> timeval;
};

vector <struct _timeval_> timefunc::timeval;

/*! -----------------------------------------------------------------------------
 * @brief 
 */
int timefunc::grep_timer (const char *timer_name)
{
    if (timeval.size() == 0)
        return -1;

    int index = 0;
    while (index < (int)timeval.size()) {
        // if (timeval[index].timer_name == timer_name)
        if (strcmp (timeval[index].timer_name, timer_name) == 0)
            return index;

        ++index;
    }

    return -1;
}

/*! -----------------------------------------------------------------------------
 * @brief 
 * @return  -1: timer ist schon vorhanden
 *          -2: Max. Anzahl timer ist erreicht. Timer kann nicht angelegt werden.
 */
int timefunc::start_timer (const char *timer_name)
{
    if (grep_timer (timer_name) >= 0) {     // timer ist schon vorhanden
        char s[256];
        sprintf (s, "start_timer(): <%s> ist schon belegt", timer_name);
        error_log::add_log (s, 0x0101, error_log::warning);
        return -1;
    }

    if (timeval.size() >= MAX_TIMER) {      // Max. Anzahl Timer wird überschritten
        error_log::add_log ("start_timer(): Max.Anzahl Timer wird ueberschritten", 0x0102, error_log::warning);
        return -2;
    }

    struct _timeval_ foo;
    strcpy (foo.timer_name, timer_name);
    gettimeofday (&foo.start, NULL);    // Startzeit merken.
    foo.dauer = 0;

    timeval.push_back(foo);

#ifdef AUSGABE_TIMER_TEXT    
    cout << "start_timer(): " << timer_name << " gestartet\n";
#endif

    return 0;
}

/*! -----------------------------------------------------------------------------
 * @brief   Funktion berechnet die Zeitdifferenz zur Startzeit in ms.
 * @param   flag    default: 0. flag=0x01: timer wird nach Abfrage gelöscht.
 * @return   -1: kein Timer gefunden \n
 *          >=0: Zeitdifferenz in ms
 */
int timefunc::get_time (const char *timer_name, uint8_t flag)
{
    int index;
    struct timeval stop;
    int diff;

    if ((index = grep_timer (timer_name)) < 0) {    // timer ist NICHT vorhanden
        char s[256];
        sprintf (s, "get_timer(): <%s> nicht gefunden", timer_name);
        error_log::add_log (s, 0x0103, error_log::warning);
        return -1;
    }

    gettimeofday (&stop, NULL);
    diff = timefunc::difference_milli (&timeval[index].start, &stop);    // Zeitdifferenz zur Startzeit in [ms]

    if (flag & 0x01)
        timeval.erase(timeval.begin() + index);     // timer löschen.

    return diff;
}

/*! -----------------------------------------------------------------------------
 * @brief   Funktion berechnet die Zeitdifferenz zur Startzeit in ms.
 *          Anschliessend wird der timer gelöscht.
 * @return   -1: kein Timer gefunden \n
 *          >=0: Zeitdifferenz in ms
 */
int timefunc::stop_timer (const char *timer_name)
{
    int diff = get_time (timer_name, 0x01);

#ifdef AUSGABE_TIMER_TEXT    
    if (diff >= 0)
        cout << "stop_timer(): " << timer_name << " = " << diff << " ms" << endl;
#endif

    return diff;
}

/*! -----------------------------------------------------------------------------
 * @brief   Funktion berechnet die Zeitdifferenz zur Startzeit in ms.
 *          Der timer wird NICHT gelöscht.
 * @return   -1: kein Timer gefunden \n
 *          >=0: Zeitdifferenz in ms
 */
int timefunc::get_timer (const char *timer_name)
{
    int diff = get_time (timer_name, 0x00);

#ifdef AUSGABE_TIMER_TEXT
    if (diff >= 0)
        cout << "get_timer(): " << timer_name << " = " << diff << " ms" << endl;
#endif

    return diff;
}

/*! -----------------------------------------------------------------------------
 * @brief Funktion list registrierte Timer auf.
 */
void timefunc::list_timer()
{
    cout << "----- Timer Liste -----\n";
    for (int i=0; i<(int)timeval.size(); i++)
        cout << timeval[i].timer_name << endl;

    cout << "----- Ende Liste -----\n";
}

/*! -----------------------------------------------------------------------------
 *  @brief  Funktion berechnet die Differenz aus <*stop> - <*start> in Microsekunden
 *	@return	Zeitdifferenz im Microsekunden. Max 35min \n\n
 *  @code   
 *          // Beispiel
 *          gettimeofday (&start, NULL);    // Startzeit merken
 *          .
 *          .
 *          gettimeofday (&stop, NULL);     // Stopzeit merken
 *          int delta = difference_micro (&start, &stop);   // Differenz berechnen
 *  @endcode
 */
int timefunc::difference_micro (struct timeval *start, struct timeval *stop)
{
	return ((signed long long) stop->tv_sec * 1000000ll +
	       (signed long long) stop->tv_usec) -
	       ((signed long long) start->tv_sec * 1000000ll +
	       (signed long long) start->tv_usec);
}   

/*!	--------------------------------------------------------------------
 *  @brief  Funktion berechnet die Differenz aus <*stop> - <*start> in Millisekunden
 *	@return	Zeitdifferenz im Millisekunden. Max 385 Std. \n\n
 *
 *  @code
 *          // Beispiel
 *          gettimeofday (&start, NULL);
 *          .
 *          .
 *          gettimeofday (&stop, NULL);
 *          int delta = difference_milli (&start, &stop);
 *  @endcode
 */
int timefunc::difference_milli (struct timeval *start, struct timeval *stop)
{
	return ((signed long long) stop->tv_sec * 1000ll +
	       (signed long long) stop->tv_usec / 1000ll) -
	       ((signed long long) start->tv_sec * 1000ll +
	       (signed long long) start->tv_usec / 1000ll);
}

//! @} timefunc

#endif